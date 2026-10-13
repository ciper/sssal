Stand alone high speed K-Line logging to an sdcard using an arduino. Canbus support is a work in progress. Although no Romraider Logger code was used directly it was an essential guide to get "fast polling" of ecu data. That project can be found at https://github.com/RomRaider/RomRaider

## Firmware — alpha8.3

Standalone SSM (K-Line) data logger for the **Arduino Uno R4 Minima**. Reads RomRaider
`logger.xml` profiles + `logger_*.xml` definitions from the SD card, builds a complete
per-ECU parameter dictionary, and logs to CSV via SSM fast-poll (continuous) batch reads.
Auto-generates a profile when the ECU is unknown or no `logger.xml` is present.

- **Source:** `firmware/sssal/sssal.ino`
- **Compiled binary:** `firmware/binaries/sssal_alpha83.bin` / `.hex`
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
  `IDLE_WAKE` history survives a long park. **On-car verified 2026-10-12 (on alpha8.1):** one
  boot, then 70 consecutive 2-minute standby wakes across two parks, with two separate drives
  detected and logged straight out of standby — no reboot, no lost drive.
- **alpha8.1:** stop advertising a broken disk. The MSC interface is always in the USB
  descriptor — `__USBInstallMSD()` is resolved at link time and the core assigns `install_MSD`
  during static init, before `setup()` runs — so a host always finds a mass-storage device even
  on a boot that goes to logger mode. TinyUSB only defaults to `MEDIUM NOT PRESENT` when no
  sense is already set (`if (p_msc->sense_key == 0)`), and sense data is sticky until a
  `REQUEST SENSE` clears it, so a stale key from any earlier failed command masked it and
  Windows offered to **format the card holding the only copy of the logs**. Every not-serving
  path now sets the sense explicitly (`SCSI_SENSE_NOT_READY`, ASC `0x3A`) and
  `tud_msc_capacity_cb` reports `0/0` so `READ CAPACITY` stalls with the same status — the host
  sees an empty card-reader slot, which is what the device actually is. Read/write failures
  while serving now report `MEDIUM ERROR` with a real ASC instead of a bare `-1`.
  Also fixed: a failed `enterMscMode()` used to `return` from `setup()` anyway, leaving the board
  neither serving the card nor logging, with nothing in `status.log` to explain it; it now falls
  through to logger mode.

  **Pulling logs:** the MSC-vs-logger choice is latched once at boot. Plugging a laptop into an
  already-powered board does not produce a drive — power-cycle it so it boots with the host
  attached. With standby enabled a parked board is usually asleep, and USB is deliberately not a
  standby wake source, so it will not enumerate until its next AGT1 underflow.
- **alpha8.2:** code-review fixes.
  - **A corrupt K-line frame no longer ends the log.** Any header/checksum error used to be
    treated exactly like the stream stopping, so one bad byte closed the log and restarted the
    ECU search (and `status.log` said "no data" either way). The reader now matches the
    `80 F0 10 <N+1> E8` header byte by byte, skips bad or cut-off frames, and resyncs on the next
    one; the log only closes after 5 s with no *valid* frame. Skipped frames are reported per log
    as `STREAM_ERRORS <file> bad_frames=N resync_bytes=M` (written only when nonzero; an
    occasional `bad_frames=1` at key-off is a frame cut off by the engine stopping).
  - **Second drive in one power-up no longer gains extra columns.** Hidden dependency params
    pulled in for calculated params were re-loaded as visible columns on the next engine start —
    a path that only became reachable once standby recovery worked.
  - **Nothing-to-log selections** (every selected param unsupported) write one
    `NO_FETCHABLE_PARAMS` line per boot and sleep, instead of creating an empty log on every start.
  - `BATCHMAX=` in `address_map.csv` is clamped to 48 (the probe's own ceiling); a hand-edited
    value of 84+ overflowed the request builder.
  - The K-line transceiver is put to sleep for each standby window (woken again before the probe).
    *(On-car verified 2026-10-12: two key-on cycles detected straight out of standby.)*
- **alpha8.3:** bug-sweep fixes.
  - **E-params no longer lost for 77 ROMs.** The definitions parser read lines into a 256-byte
    buffer; 53 of the `<ecu id="...">` ROM lists in `logger_IMP_EN_v370.xml` are longer (up to
    855 characters), so any ROM ID past the cut was never matched — 559 ROM×parameter pairs, up
    to 29 E-params for one ROM. The parser now has its own 1 KB buffer and reports any line that
    still overflows (`DEFS_LINE_TRUNCATED`). `address_map.csv` gains a `FMT=2` header line, so a
    cache built by the old parser is rebuilt once (`CACHE_REGEN reason=BUILDER_UPDATE`, ~19 s).
  - **A failed cache build can no longer stick.** The dictionary is built into
    `address_map.tmp` and only renamed into place if the whole SSM section was read and written;
    previously a defs read error or power loss mid-build left an empty/partial cache that was
    reused forever. Failures log `CACHE_BUILD_FAIL` and retry at the next wake instead of halting.
  - **No more 50 MB files after a power cut.** SdFat's `preAllocate()` sets the file size to the
    full reservation immediately, so a log that never closed was 50 MB of rows plus stale card
    data. Pre-allocation is removed (at ~0.5 KB/s it bought nothing); an unclosed log now ends at
    the last 1 s sync.
  - **Fatal errors no longer drain the battery, and their LED pattern is actually visible.** The
    halt loop ran awake forever (~15 mA). It also blinked an LED that could not light: D13 is the
    SD card's SPI clock, and `digitalWrite()` doesn't take a pin back from a peripheral. Fatal
    states now flash a visible pattern for 60 s (4 blinks = config error, reason in `status.log`;
    6 blinks = SD card failed), then sleep in Software Standby and repeat the pattern every ~4 min.
    An empty profile is no longer fatal — it is reported as `NO_FETCHABLE_PARAMS` and parks.
  - **`NO_FETCHABLE_PARAMS` is now really quiet:** later wakes that find the same ECU go back to
    sleep instead of re-running detection and re-writing its status lines every 2 minutes.
  - **Wake-timer re-arm waits for the timer to stop** before reconfiguring it, as Renesas' FSP
    driver does. It worked before only because every re-arm wrote identical values.

### Pre-flight tool (`tools/SsmInitTiming.cs`)
A PC-side J2534 validator (Tactrix OpenPort 2.0) that checks everything the firmware relies
on **before** flashing: ROM-ID + capability decode, capability→param-name mapping, live
param/switch/E-param (extended-address) reads, max-batch probe, single vs continuous
(fast-poll) timing. Run `SsmInitTiming.exe`; `SsmInitTiming.exe brake` watches a switch
toggle live. Validated on-car against an EJ ECU (ROM `2F12515506`).
