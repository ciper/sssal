# Build notes

## Toolchain
- `arduino-cli` with core **`arduino:renesas_uno@1.5.3`**
- Libraries: **SdFat 2.3.0**, **RTClib 2.1.4**, **Arduino_CAN** (bundled with the core)
- Compile: `arduino-cli compile --fqbn arduino:renesas_uno:minima firmware/sssal`
- Upload: `arduino-cli upload -p <COMx> --fqbn arduino:renesas_uno:minima firmware/sssal`
  (Uno R4 Minima enumerates as a USB CDC COM port; upload resets it into DFU automatically.)

## Required core patches for USB Mass Storage (v5)

The USB-MSC feature (SD exposed as a USB drive) needs **two edits to the Arduino core**. They
live inside the installed core, **not** in this repo, so they are **wiped by any core update**
and must be re-applied after upgrading. Core path:
`<Arduino15>/packages/arduino/hardware/renesas_uno/1.5.3/`

### 1. Enable the MSC device class
`variants/MINIMA/tusb_config.h` — the TinyUSB MSC class ships disabled:

```c
-#define CFG_TUD_MSC              0
+#define CFG_TUD_MSC              1
```

CDC (`CFG_TUD_CDC`) stays `1`, so USB serial still works alongside MSC.

### 2. Fix the MSC endpoint max-packet-size (real upstream bug)
`cores/arduino/usb/USB.cpp` — the MSC bulk endpoint size is hardcoded to 512, which is only
legal on high-speed USB. The RA4M1 is **full-speed** (max 64), so 512 produces an invalid
endpoint descriptor and the whole composite device fails to start (Windows "problem code 10").
Guard it to 64 for full-speed, mirroring how CDC already does it:

```c
-#define USBD_MSD_IN_OUT_SIZE (512)
+#if (CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE)
+#define USBD_MSD_IN_OUT_SIZE (512)
+#else
+#define USBD_MSD_IN_OUT_SIZE (64)
+#endif
```

This is a latent bug in `ArduinoCore-renesas` that was never exercised because MSC ships
disabled. Worth reporting upstream.

### After editing a core file
`arduino-cli` caches the compiled core separately from the sketch build, so a core-file edit
is not picked up by `compile --clean` alone. Clear the cache first:

```sh
arduino-cli cache clean
arduino-cli compile --clean --fqbn arduino:renesas_uno:minima firmware/sssal
```

### Sketch side (already in sssal.ino)
Enabling the class is not enough — the core only advertises MSC in the USB descriptor if the
sketch defines the weak presence function `void __USBInstallMSD() {}` (C++ linkage, no
`extern "C"`). The sketch also implements the `tud_msc_*` callbacks against the SdFat block
device and latches MSC-vs-logger mode at boot via `tud_mounted()`.
