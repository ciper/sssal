Stand alone high speed K-Line logging to an sdcard using an arduino. Canbus support is a work in progress. Although no Romraider Logger code was used directly it was an essential guide to get "fast polling" of ecu data. That project can be found at https://github.com/RomRaider/RomRaider

## Firmware — Alpha v3

Standalone SSM (K-Line) data logger for the **Arduino Uno R4 Minima**. Reads RomRaider
`logger.xml` profiles + `logger_*.xml` definitions from the SD card, builds a complete
per-ECU parameter dictionary, and logs to CSV via SSM fast-poll (continuous) batch reads.
Auto-generates a profile when the ECU is unknown or no `logger.xml` is present.

- **Source:** `firmware/sssal/sssal.ino`
- **Compiled binaries:** `firmware/binaries/sssal_alpha3.bin` and `sssal_alpha3.hex`
- **Build:** `arduino-cli compile --fqbn arduino:renesas_uno:minima` (libs: SdFat, RTClib)
- **Flash:** `arduino-cli upload` or bossac (Renesas RA4M1)

### Changes since Alpha v2
- **Per-ECU batch-read limit:** the firmware now probes each ECU's max SSM batch size on
  first detection, caches it in the `address_map.csv` header (`BATCHMAX=`), and reuses it on
  later boots (no re-probe). If a selection exceeds the limit, the **highest-numbered**
  params are dropped and logged to `status.log` (`BATCH_LIMIT ...`), instead of silently
  failing. (On the test ECU the real ceiling was ~40 addresses, vs the old assumed 84.)
- **Timing constants** updated from on-car J2534 measurement (init ~148 ms round-trip).

### Pre-flight tool (`tools/SsmInitTiming.cs`)
A PC-side J2534 validator (Tactrix OpenPort 2.0) that checks everything the firmware relies
on **before** flashing: ROM-ID + capability decode, capability→param-name mapping, live
param/switch/E-param (extended-address) reads, max-batch probe, single vs continuous
(fast-poll) timing. Run `SsmInitTiming.exe`; `SsmInitTiming.exe brake` watches a switch
toggle live. Validated on-car against an EJ ECU (ROM `2F12515506`).

> **Alpha — firmware not yet hardware-tested** (compiles clean: Flash 39%, RAM 50%). The
> SSM protocol paths are bench-validated via the pre-flight tool.
