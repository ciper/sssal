Stand alone high speed K-Line logging to an sdcard using an arduino. Canbus support is a work in progress. Although no Romraider Logger code was used directly it was an essential guide to get "fast polling" of ecu data. That project can be found at https://github.com/RomRaider/RomRaider

## Firmware — alpha8.0

Standalone SSM (K-Line) data logger for the **Arduino Uno R4 Minima**. Reads RomRaider
`logger.xml` profiles + `logger_*.xml` definitions from the SD card, builds a complete
per-ECU parameter dictionary, and logs to CSV via SSM fast-poll (continuous) batch reads.
Auto-generates a profile when the ECU is unknown or no `logger.xml` is present.

- **Source:** `firmware/sssal/sssal.ino`
- **Compiled binary:** `firmware/binaries/sssal_alpha8.bin` / `.hex`
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
- **alpha6:** standby-recovery fix. After an engine-off stream-watchdog the logger re-enters
  Phase 1; the low-power idle path (Software Standby) previously never recovered when the OBD
  port stays powered (nothing power-cycles the board). Now re-inits the UART + transceiver on
  every wake before probing, and emits an `IDLE_WAKE count=N` status line so a log can tell
  "woke, found nothing" from "never woke". `status.log` rotates to `status.1.log` at 2 KB
  instead of being deleted, so the close/wake history survives a reboot. *(Flashed; on-car
  verification of the standby path pending.)*
- **alpha7.0:** RomRaider-aligned log format. Each log opens with a `#`-prefixed provenance
  remark (`# Created by SSSAL <ver> | ROM <id> | <datetime> | defs <file>`); the time column is
  now `Time (msec)` and is **zeroed per log** (first row = 0) to match RomRaider's relative
  timestamp; value-column headers are `Name (units) [ID]`. Data values keep full SSSAL
  resolution (uniform 2 decimals — not rounded to RomRaider's per-column `format`).
- **alpha8.0:** **low-power Software Standby wake now works.** The parked idle path had never
  recovered: the board entered Software Standby and stayed there, so with a continuously-powered
  OBD port (nothing power-cycles the logger) every drive after an engine-off was lost. Two real
  defects in the AGT1 wake-timer setup, both found by measuring registers on-car rather than by
  inspection:
  1. **Wrong register offsets.** `AGTMR1`/`AGTMR2` were addressed at `+0x18`/`+0x19` instead of
     `+0x09`/`+0x0A` (verified against the RA4M1 SVD). The names `AGTIOC`/`AGTISR` pointed at the
     *real* mode registers, so the "clear the block" step zeroed them and the intended config
     writes landed outside the peripheral. `AGTMR1.TCK` stayed `000` = PCLKB — a clock that stops
     in Software Standby — so the timer froze the moment the board slept.
  2. **`AGTMR2.LPM` was set while the timer was stopped.** That prevents the counter from ever
     starting: `AGTCR.TCSTF` never sets and the count stays frozen. Renesas' own FSP `r_agt.c`
     sets only the `CKS` divider before start, starts with `AGTCR = 0xF1` (the status flags are
     write-0-to-clear, so `0x01` silently clears them), and waits for `TCSTF == 1` *before*
     modifying any other AGT register. LPM must be enabled only after that.

  Bench-proven on hardware: a 10 s standby returned `rtc_s=10` on the external DS1307 with
  `millis_s=0` (the MCU clock did not advance — genuine Software Standby), one AGT1 underflow
  wake, then stable operation. Parked draw drops from ~7.5 mA to ~40 µA. `STANDBY_ENABLED` is a
  compile-time switch; set it to `0` for the fast-search build that stays awake (~7.5 mA parked)
  if a standby problem is ever suspected. `status.log` now rotates at 32 KB instead of 2 KB so the
  `IDLE_WAKE` history survives a long park. *(On-car key-off verification still pending.)*

### Pre-flight tool (`tools/SsmInitTiming.cs`)
A PC-side J2534 validator (Tactrix OpenPort 2.0) that checks everything the firmware relies
on **before** flashing: ROM-ID + capability decode, capability→param-name mapping, live
param/switch/E-param (extended-address) reads, max-batch probe, single vs continuous
(fast-poll) timing. Run `SsmInitTiming.exe`; `SsmInitTiming.exe brake` watches a switch
toggle live. Validated on-car against an EJ ECU (ROM `2F12515506`).
