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

### MSC is always advertised — why the callbacks must report "no medium"
`__USBInstallMSD()` is resolved at **link** time and the core assigns `install_MSD` during
static init, *before* `setup()` runs. The MSC interface is therefore always in the USB
descriptor, including on a boot that goes to logger mode, and there is no way to gate the
descriptor at runtime without patching the core further. A host always finds a mass-storage
device, whether or not the sketch intends to serve the card.

TinyUSB does default to `NOT READY / MEDIUM NOT PRESENT` on a failed `TEST UNIT READY`, but
only `if (p_msc->sense_key == 0)` — see `tinyusb/class/msc/msc_device.c` lines 667, 704 and
740. Sense data is sticky until a `REQUEST SENSE` clears it, so any earlier failed command
leaves a stale key that masks "no medium". The host then treats the disk as present but
unreadable and offers to **format the card holding the only copy of the logs**.

So every not-serving path in the `tud_msc_*` callbacks sets the sense explicitly:

```c
tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);   // 0x3A = MEDIUM NOT PRESENT
```

and `tud_msc_capacity_cb` reports `block_count = 0, block_size = 0` so the core stalls
`READ CAPACITY` with the same status. The host then shows an empty card-reader slot, which is
exactly what the device is at that moment.

### Pulling logs off the card
The MSC-vs-logger decision is latched **once**, at boot, and never re-evaluated. Plugging a
laptop into a board that is already powered (from OBD or a charger) does **not** produce a
drive — the board stays in logger mode and reports no medium. Power-cycle the board so it boots
with the host attached.

With `STANDBY_ENABLED 1` the board is also usually in Software Standby when parked, and USB is
**not** a standby wake source (`WUPEN` bit 27 is deliberately left unset — only bit 28, the AGT1
underflow, is enabled). A sleeping board will not enumerate on USB at all until its next AGT1
underflow, and will then wake into logger mode. Power-cycling is the only reliable path.
