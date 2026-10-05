Stand alone high speed K-Line logging to an sdcard using an arduino. Canbus support is a work in progress. Although no Romraider Logger code was used directly it was an essential guide to get "fast polling" of ecu data. That project can be found at https://github.com/RomRaider/RomRaider

## Firmware — Alpha v5

Standalone SSM (K-Line) data logger for the **Arduino Uno R4 Minima**. Reads RomRaider
`logger.xml` profiles + `logger_*.xml` definitions from the SD card, builds a complete
per-ECU parameter dictionary, and logs to CSV via SSM fast-poll (continuous) batch reads.
Auto-generates a profile when the ECU is unknown or no `logger.xml` is present.

- **Source:** `firmware/sssal/sssal.ino`
- **Compiled binary:** `firmware/binaries/sssal_alpha5.bin` / `.hex`
- **Build:** `arduino-cli compile --fqbn arduino:renesas_uno:minima` (libs: SdFat 2.3.0, RTClib 2.1.4, Arduino_CAN). **USB-MSC requires two one-line core patches — see [`firmware/BUILD_NOTES.md`](firmware/BUILD_NOTES.md).**
- **Flash:** `arduino-cli upload -p <COMx> --fqbn arduino:renesas_uno:minima` (Renesas RA4M1, DFU).

> **★ Hardware-validated on-car (2026-10-05).** First on-car run on an EJ255 (ROM `2F12515506`)
> detected the ECU, resolved 55 params, and logged ~96k clean closed-loop rows. Earlier alphas
> were bench-validated only via the pre-flight tool; the logger path now has a real on-car run.

### USB Mass Storage (SD-as-USB-drive) — new in v5
Plug the logger into a PC and it enumerates as a USB flash drive, exposing the SD card so you
can copy logs off and drop config files on **without removing the microSD**. Mode is chosen at
boot: a USB host present → USB-drive mode; powered from the car with no host → normal logger.
This needs the two core patches in `BUILD_NOTES.md` (one of which fixes a real upstream core bug
in the full-speed MSC endpoint descriptor).

### Changes since Alpha v3
- **v4:** SSM-over-CAN transport stubbed in with a byte-exact ISO15765 spec (seam only, not yet
  active); ECU-init buffer 96→112 bytes; default profile tweaks.
- **v5:** USB Mass Storage (above); RTC set-on-boot via a one-shot `/logger/settime.txt`
  (`YYYY-MM-DD HH:MM:SS`) with a compile-time fallback when the RTC is unset; FAT directory
  timestamps now track the RTC (SdFat date/time callback) so file-property dates match the
  filename; K-line transceiver sleep pin (D2) driven awake at boot.

### Pre-flight tool (`tools/SsmInitTiming.cs`)
A PC-side J2534 validator (Tactrix OpenPort 2.0) that checks everything the firmware relies
on **before** flashing: ROM-ID + capability decode, capability→param-name mapping, live
param/switch/E-param (extended-address) reads, max-batch probe, single vs continuous
(fast-poll) timing. Run `SsmInitTiming.exe`; `SsmInitTiming.exe brake` watches a switch
toggle live. Validated on-car against an EJ ECU (ROM `2F12515506`).
