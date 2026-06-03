Stand alone high speed K-Line logging to an sdcard using an arduino. Canbus support is a work in progress. Although no Romraider Logger code was used directly it was an essential guide to get "fast polling" of ecu data. That project can be found at https://github.com/RomRaider/RomRaider

## Firmware — Alpha v2

Standalone SSM (K-Line) data logger for the **Arduino Uno R4 Minima**. Reads RomRaider
`logger.xml` profiles + `logger_*.xml` definitions from the SD card, builds a complete
per-ECU parameter dictionary, and logs to CSV. Auto-generates a profile when the ECU is
unknown or no `logger.xml` is present.

- **Source:** `firmware/sssal/sssal.ino`
- **Compiled binaries:** `firmware/binaries/sssal_alpha2.bin` and `sssal_alpha2.hex`
- **Build:** `arduino-cli compile --fqbn arduino:renesas_uno:minima` (libs: SdFat, RTClib)
- **Flash:** `arduino-cli upload` or bossac (Renesas RA4M1)

> **Alpha — not yet hardware-tested.** Compiles clean (Flash 39%, RAM 50%).
