// ============================================================
// SSSAL — Subaru SSM SDCard Arduino Logger
// Hardware: Arduino Uno R4 Minima + TJA1020 K-Line transceiver
//           + HiLetgo Data Logger Shield (SD + DS1307 RTC)
// Reads RomRaider logger.xml profile and logger_*.xml definitions
// from SD card; auto-generates address cache per ECU; logs SSM
// data to timestamped CSV files.
// ============================================================

#include <SdFat.h>
#include <RTClib.h>
#include <Arduino_CAN.h>      // RA4M1 CAN FD — used for OBD protocol (pins D4/D5)
#include <math.h>            // fmodf() for the % operator in the expression evaluator

// ============================================================
// === CONFIG ===
// ============================================================

// -- Hardware pins --
#define SD_CS_PIN            10     // HiLetgo logger shield SD chip select (SPI)
// LED_BUILTIN = 13                 // already defined by R4 Minima board package

// -- Baud rates --
#define SERIAL_DEBUG_BAUD    115200 // USB CDC (Serial) — debug output only
#define SSM_BAUD             4800   // K-Line / SSM — Serial1 (pins 0 RX / 1 TX)

// -- SSM timing --
// PHASE1_RETRY_MS: placeholder — update after SsmInitTiming.exe run on car
// floor ~155ms (12.5ms TX + 12.5ms echo + ~129ms for ~62-byte response); 300ms buffers
#define PHASE1_RETRY_MS            300    // ms to wait for init response first byte
#define PHASE1_SEARCH_DURATION_MS  120000 // 2 min of fast retries before sleeping (PHASE1_SEARCH)
#define PHASE1_IDLE_SLEEP_MS       120000 // 2 min sleep between retries when parked (PHASE1_IDLE)
#define PHASE2_WATCHDOG_MS         5000   // ms without streaming data → close log, re-enter PHASE1_SEARCH

// -- SD write performance --
#define WRITE_BUF_SIZE       512    // CSV write buffer; matches SD sector size
#define PREALLOCATE_BYTES    (1024UL * 1024 * 50) // 50MB reserved upfront; avoids FAT chain updates
#define SYNC_INTERVAL_MS     1000   // file.sync() every 1s; ~1-2ms at 25MHz SPI

// -- status.log --
#define STATUS_LOG_MAX_BYTES 2048   // rotate (delete) status.log when it exceeds this size

// -- Session counter --
#define COUNTER_MAX          9999   // wrap to 1 after reaching this value

// -- Buffer and array limits --
#define LINE_BUF_SIZE        256    // max line length in XML/CSV files
#define MAX_SELECTED         32     // max params selectable in logger.xml
#define MAX_FETCH            48     // max addresses in SSM batch (selected + CALCULATED deps)
#define MAX_BATCH_DATA       192    // worst-case data bytes per batch response (MAX_FETCH×4)
#define MAX_SSM_ADDR         84     // SSM single-request address limit: (255-2)/3 = 84
                                    // (one address per data byte; ECU returns 1 byte/address)
#define MAX_EXPR_LEN         96     // max chars in a formula expr= (OBD if() exprs reach ~71)
#define MAX_NAME_LEN         48     // max characters in a param name
#define MAX_ID_LEN           24     // max characters in a param ID (e.g. E_IMMO_4AD9_4ADB)
#define MAX_DEPS             8      // max dep params per CALCULATED param

// -- File paths --
#define LOGGER_DIR           "/logger"
#define PROFILE_FILE         "/logger/logger.xml"
#define UNKNOWNECU_FILE      "/logger/unknownecu.xml"
#define ADDR_MAP_FILE        "/logger/address_map.csv"
#define STATUS_LOG_FILE      "/logger/status.log"
#define COUNTER_FILE         "/logger/counter.txt"
#define VERBOSE_LOG_FILE     "/logger/verboselog.txt"  // created when DEBUG=true in address_map.csv

// ============================================================
// === GLOBALS ===
// ============================================================

// --- Type constants (ptype field) ---
#define T_U8    0   // unsigned  1-byte int
#define T_U16   1   // unsigned  2-byte int
#define T_U32   2   // unsigned  4-byte int
#define T_I8    3   // signed    1-byte int
#define T_I16   4   // signed    2-byte int
#define T_I32   5   // signed    4-byte int
#define T_FLOAT 6   // IEEE-754  4-byte float (memcpy, never cast)
#define T_BIT   7   // 1-byte read; extract bit given by bitNum
#define T_CALC  8   // no fetch; computed from dep params in expr
#define T_NONE  9   // NOTFOUND — log 0.00 forever
#define T_HEX   10  // raw integer register dump (units="raw*", expr="x") — log as exact hex

// --- Endian constants ---
#define END_BIG    0
#define END_LITTLE 1

// --- Protocol constants ---
#define PROTO_SSM  0
#define PROTO_OBD  1   // ISO 15765 CAN — requires TJA1050 on D4/D5, 500kbps

// --- Phase constants ---
#define PHASE_SEARCH    0   // fast retry loop (up to 2 min)
#define PHASE_IDLE      1   // low-power standby (2-min sleep cycles)
#define PHASE_STREAMING 2   // ECU streaming; logging to SD

// --- Sentinel values ---
#define FETCH_NONE 0xFF     // fetchIdx value meaning "no fetch slot" (CALC / NOTFOUND)

// ---------------------------------------------------------------
// Param — one selected parameter loaded from address_map.csv
//   Total: 150 bytes × MAX_SELECTED(32) = 4800 bytes
// ---------------------------------------------------------------
struct Param {
    char     id[MAX_ID_LEN];      // param ID, e.g. "E83", "P8"
    char     name[MAX_NAME_LEN];  // human name for CSV header
    uint32_t address;             // 3-byte SSM address (upper byte always 0)
    uint8_t  len;                 // bytes to read (0 for T_CALC / T_NONE)
    uint8_t  ptype;               // T_* constant above
    uint8_t  endian;              // END_BIG or END_LITTLE
    uint8_t  bitNum;              // for T_BIT: which bit to extract (0=LSB)
    char     expr[MAX_EXPR_LEN];  // formula string; for T_CALC, may ref other param IDs
    uint8_t  fetchIdx;            // index into g_fetch[]; FETCH_NONE if T_CALC or T_NONE
    float    lastVal;             // last computed engineering value (dep resolution for T_CALC)
    bool     divZeroLogged;       // true after first DIV_ZERO event written this session
    bool     hidden;              // true for auto-added dep params — fetched but not in CSV output
};

// ---------------------------------------------------------------
// FetchSlot — one address entry in the SSM batch request
//   Total: 6 bytes × MAX_FETCH(48) = 288 bytes
// ---------------------------------------------------------------
struct FetchSlot {
    uint32_t address;   // SSM address (3 bytes significant)
    uint8_t  len;       // bytes to read at this address
    uint8_t  dataOff;   // byte offset into response data[] buffer (computed at batch build)
};

// ---------------------------------------------------------------
// Shared hardware objects
// ---------------------------------------------------------------
SdFat        g_sd;
SdFile       g_logFile;
RTC_DS1307   g_rtc;
bool         g_rtcOk = false;

// ---------------------------------------------------------------
// Parameter tables
// ---------------------------------------------------------------
Param      g_sel[MAX_SELECTED];   // selected params (from logger.xml, loaded from cache)
uint8_t    g_numSel = 0;

FetchSlot  g_fetch[MAX_FETCH];    // batch request address list
uint8_t    g_numFetch = 0;
uint8_t    g_numDataBytes = 0;    // total response data bytes (sum of all fetch lens)

// Exact assembled integer per selected param (filled at decode time for integer
// types). Used to log raw-integer params (expr="x") as exact hex — bypasses the
// float in vals[], which loses precision above 2^24 (e.g. 4-byte immo registers).
uint32_t   g_rawVals[MAX_SELECTED];

// ---------------------------------------------------------------
// Write buffer (accumulate CSV rows before SD write)
// ---------------------------------------------------------------
char     g_writeBuf[WRITE_BUF_SIZE];
uint16_t g_writeBufPos  = 0;
uint32_t g_lastSync     = 0;

// ---------------------------------------------------------------
// Session state
// ---------------------------------------------------------------
uint8_t  g_protocol    = PROTO_SSM;
uint8_t  g_phase       = PHASE_SEARCH;
uint16_t g_sessionNum  = 0;

// ROM ID of currently connected ECU (10 hex chars + NUL)
char     g_romId[11]   = {0};

// True if a logger.xml was present at boot. When false, a profile is auto-generated
// after ECU detection (default logger.xml for a known ECU, unknownecu.xml otherwise).
bool     g_haveLoggerXml = false;

// Definitions filename detected on SD (e.g. "logger_IMP_EN_v370.xml")
char     g_defsFilename[40] = {0};

// ECU init response raw buffer — kept during cache regen for capability checks
// Max practical size: 4 header + 1 cmd + 3 reserved + 5 romId + 69 cap + 1 cs = 83 bytes
// Allocate 96 for headroom.
uint8_t  g_ecuInitRaw[96];
uint8_t  g_ecuInitLen  = 0;   // rawResp[3] - 1  (data bytes minus the 0xFF command byte)

// Batch request packet (built once per engine start; reused every poll).
// Sized for one 3-byte address per data byte (multi-byte params expand to
// consecutive addresses): MAX_SSM_ADDR addresses × 3 + 7 framing bytes.
uint8_t  g_batchReq[MAX_SSM_ADDR * 3 + 7];
uint8_t  g_batchReqLen = 0;

// Flag: write terminal LOGGING_STARTED line on first packet received
bool     g_firstPacket = false;
// Flag: set true if SD write fails mid-session; checked in loop() to close log + re-search
bool     g_sdWriteError = false;
// Filename of the current open log file (set when file opened, used for status.log)
char     g_logFilename[28] = {0};
// Verbose debug logging: set from the DEBUG= line at the top of address_map.csv.
// When true, vlog() appends a timestamped line to verboselog.txt at each call site.
bool     g_verboseDebug = false;

// ============================================================
// === SSM PROTOCOL ===
// ============================================================
// All SSM I/O uses Serial1 (pins 0 RX / 1 TX, 4800 8N1).
// K-Line is half-duplex: every TX byte echoes back on RX —
// caller must discard the echo after every send.
//
// Init request  (6 bytes):  80 10 F0 01 BF 40
// Init response (variable): 80 F0 10 <len> FF <3 reserved> <5 ROM-ID> <cap bytes...> <cs>
// Batch request (variable): 80 10 F0 <N*3+2> A8 <poll> <addr×3 each> <cs>
// Batch response(variable): 80 F0 10 <N+1>   E8 <data bytes> <cs>
//
// Checksum: sum of ALL bytes [0..len-2] (including 0x80 header), mod 256.
// ============================================================

// ---- Low-level helpers ----

// Read one byte from Serial1 with a millisecond timeout.
// Returns the byte (0-255) on success, -1 on timeout.
static int ssmReadByte(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (!Serial1.available()) {
        if (millis() - start >= timeoutMs) return -1;
    }
    return Serial1.read();
}

// Discard n bytes from Serial1 (echo drain after a send).
static void ssmDiscardEcho(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        ssmReadByte(50);   // 50ms per byte; at 4800 baud a byte takes 2.08ms
    }
}

// Flush all pending bytes from Serial1 RX buffer.
// Call before re-entering PHASE_SEARCH to clear partial stream data.
static void ssmFlushRx() {
    while (Serial1.available()) Serial1.read();
}

// Compute SSM checksum: sum of buf[0..len-2], mod 256.
static uint8_t ssmChecksum(const uint8_t *buf, uint8_t len) {
    uint16_t total = 0;
    for (uint8_t i = 0; i < len - 1; i++) total += buf[i];
    return (uint8_t)(total & 0xFF);
}

// ---- Init packet ----

// Send the 6-byte SSM init request and discard the 6-byte echo.
// Does NOT wait for the ECU response — caller handles that.
static void ssmSendInitPacket() {
    const uint8_t pkt[6] = {0x80, 0x10, 0xF0, 0x01, 0xBF, 0x40};
    Serial1.write(pkt, 6);
    Serial1.flush();         // wait until TX hardware drains
    ssmDiscardEcho(6);       // drain matching echo bytes
}

// Read the ECU init response into rawResp[0..].
// Returns true on success; false on timeout or checksum error.
// On success, rawResp[] holds the complete response and *respTotalLen is set.
// Fills g_romId[0..10] and g_ecuInitRaw[] / g_ecuInitLen on success.
static bool ssmReadInitResponse(uint8_t *rawResp, uint8_t maxLen, uint8_t *respTotalLen) {
    // Read 4-byte header: [0]=0x80 [1]=0xF0 [2]=0x10 [3]=data_len
    for (uint8_t i = 0; i < 4; i++) {
        int b = ssmReadByte(PHASE1_RETRY_MS);
        if (b < 0) return false;
        rawResp[i] = (uint8_t)b;
    }
    if (rawResp[0] != 0x80 || rawResp[1] != 0xF0 || rawResp[2] != 0x10) return false;

    uint8_t dataLen = rawResp[3];              // includes 0xFF command byte
    // Minimum valid SSM init response: 0xFF + 3 reserved + 5 ROM-ID = 9 data bytes
    if (dataLen < 9) return false;
    // Use uint16_t: dataLen is uint8_t so dataLen+1 can wrap to 0 at value 255
    uint16_t remaining = (uint16_t)dataLen + 1;  // data bytes + checksum byte
    if ((uint16_t)4 + remaining > (uint16_t)maxLen) return false;

    for (uint16_t i = 0; i < remaining; i++) {
        int b = ssmReadByte(100);
        if (b < 0) return false;
        rawResp[4 + i] = (uint8_t)b;
    }

    uint8_t total = (uint8_t)(4 + remaining);  // safe: bounds check above ensures <= maxLen
    *respTotalLen = total;

    // Verify checksum
    if (rawResp[total - 1] != ssmChecksum(rawResp, total)) return false;

    // Extract 5-byte ROM ID from rawResp[8..12]
    // ecuInitBytes = rawResp[5..]; ROM ID at ecuInitBytes[3..7] = rawResp[8..12]
    snprintf(g_romId, sizeof(g_romId), "%02X%02X%02X%02X%02X",
             rawResp[8], rawResp[9], rawResp[10], rawResp[11], rawResp[12]);

    // Store capability bytes for cache regen: ecuInitLen = dataLen - 1 (minus 0xFF cmd byte)
    uint8_t ecuLen = dataLen - 1;
    uint8_t copyLen = ecuLen < sizeof(g_ecuInitRaw) ? ecuLen : sizeof(g_ecuInitRaw);
    memcpy(g_ecuInitRaw, rawResp + 5, copyLen);
    // Bound g_ecuInitLen to what actually fits in g_ecuInitRaw[] so ssmParamSupported()
    // never indexes past the buffer (real defs max ecubyteindex=76 < 96, so no behaviour
    // change in practice — this only hardens against a malformed/oversized response).
    g_ecuInitLen = copyLen;

    return true;
}

// Check whether a P param's capability bit is set in the stored init response.
// ecuByteIndex and ecuBit are taken directly from the definitions file attributes.
// Returns true if supported, false if not supported or index out of range.
bool ssmParamSupported(uint8_t ecuByteIndex, uint8_t ecuBit) {
    if (ecuByteIndex >= g_ecuInitLen) return false;
    return (g_ecuInitRaw[ecuByteIndex] & (1 << ecuBit)) != 0;
}

// ---- Batch request ----

// Build the SSM batch read request from g_fetch[] into g_batchReq[].
// Sets g_batchReqLen, g_numDataBytes, and g_fetch[i].dataOff for each slot.
// pollFlag: 0x00 = read once, 0x01 = fast poll (continuous stream).
static void ssmBuildBatchRequest(uint8_t pollFlag) {
    // dataOff: running tally of byte offset into the response data buffer
    uint8_t dataOff = 0;
    for (uint8_t i = 0; i < g_numFetch; i++) {
        g_fetch[i].dataOff = dataOff;
        dataOff += g_fetch[i].len;
    }
    g_numDataBytes = dataOff;   // total data bytes == total addresses (1 byte per address)

    // SSM read-address (0xA8) returns ONE byte per requested 3-byte address, so a
    // multi-byte param of length N must request N CONSECUTIVE addresses
    // (base, base+1, ... base+N-1). The response then has N bytes for that param.
    //
    // Packet: 80 10 F0 <len_byte> A8 <pollFlag> <addr×3 each> <cs>
    // len_byte = (#addresses)*3 + 2  (cmd A8 + pollFlag + all address bytes)
    // #addresses == g_numDataBytes (capped at MAX_SSM_ADDR=84 by buildFetchList).
    uint8_t lenByte = (uint8_t)(g_numDataBytes * 3 + 2);
    uint8_t idx = 0;
    g_batchReq[idx++] = 0x80;
    g_batchReq[idx++] = 0x10;
    g_batchReq[idx++] = 0xF0;
    g_batchReq[idx++] = lenByte;
    g_batchReq[idx++] = 0xA8;
    g_batchReq[idx++] = pollFlag;
    for (uint8_t i = 0; i < g_numFetch; i++) {
        for (uint8_t b = 0; b < g_fetch[i].len; b++) {
            uint32_t a = g_fetch[i].address + b;
            g_batchReq[idx++] = (uint8_t)((a >> 16) & 0xFF);
            g_batchReq[idx++] = (uint8_t)((a >>  8) & 0xFF);
            g_batchReq[idx++] = (uint8_t)( a        & 0xFF);
        }
    }
    g_batchReq[idx] = ssmChecksum(g_batchReq, idx + 1);
    g_batchReqLen = idx + 1;
}

// Send g_batchReq[] and discard the echo.
static void ssmSendBatchRequest() {
    Serial1.write(g_batchReq, g_batchReqLen);
    Serial1.flush();
    ssmDiscardEcho(g_batchReqLen);
}

// ---- Batch response ----

// Read one batch response into data[0..g_numDataBytes-1].
// Validates header bytes and checksum.
// Returns true on success, false on timeout or error.
// timeoutMs: how long to wait for the first byte.
static bool ssmReadBatchResponse(uint8_t *data, uint32_t timeoutMs) {
    // Response: 80 F0 10 <N+1> E8 <N data bytes> <cs>
    // Total bytes = g_numDataBytes + 6
    uint8_t totalLen = g_numDataBytes + 6;
    uint8_t buf[MAX_BATCH_DATA + 6];   // worst case: 192 data bytes + 6 framing

    // First byte with caller-specified timeout (watchdog)
    int b = ssmReadByte(timeoutMs);
    if (b < 0) return false;
    buf[0] = (uint8_t)b;

    // Remaining bytes with generous per-byte timeout
    for (uint8_t i = 1; i < totalLen; i++) {
        b = ssmReadByte(200);
        if (b < 0) return false;
        buf[i] = (uint8_t)b;
    }

    // Validate header and data-length field (buf[3] = g_numDataBytes + 1 for 0xE8 command)
    if (buf[0] != 0x80 || buf[1] != 0xF0 || buf[2] != 0x10 ||
        buf[3] != (uint8_t)(g_numDataBytes + 1) || buf[4] != 0xE8) return false;

    // Validate checksum
    if (buf[totalLen - 1] != ssmChecksum(buf, totalLen)) return false;

    // Copy data bytes (buf[5..5+N-1]) to caller's buffer
    memcpy(data, buf + 5, g_numDataBytes);
    return true;
}

// ---- Decode a raw fetch slot into a float engineering value ----
// Reads len bytes from data[dataOff..], assembles per type/endian, applies expr.
// For T_BIT: reads 1 byte, extracts bit bitNum.
// For T_FLOAT: memcpy 4 bytes into float, then eval expr.
// For T_CALC / T_NONE: returns 0.0f (caller handles CALC).
// Forward-declares evalExpr() which is defined in the EXPRESSION EVALUATOR section.
float evalExpr(const char *expr, float x);   // forward declaration

// Assemble `len` bytes (big-endian after endian handling) into the EXACT uint32 —
// no float, no formula. Mirrors ssmDecodeSlot's integer assembly. Used to log
// raw-integer params (expr="x") as exact hex so multi-byte immo registers keep
// every bit (float would round values above 2^24).
static uint32_t assembleRawInt(const uint8_t *data, uint8_t dataOff,
                               uint8_t len, uint8_t endian) {
    if (len == 0) return 0;
    if (len > 4) len = 4;
    uint8_t raw[4] = {0};
    if (endian == END_BIG)
        for (uint8_t i = 0; i < len; i++) raw[i] = data[dataOff + i];
    else
        for (uint8_t i = 0; i < len; i++) raw[len - 1 - i] = data[dataOff + i];
    uint32_t u = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16)
               | ((uint32_t)raw[2] <<  8) |  (uint32_t)raw[3];
    return u >> ((4 - len) * 8);
}

// True if a param is logged as exact hex. The decision (integer storagetype +
// raw passthrough expr="x" + units beginning "raw") is made once at cache
// generation and encoded as the T_HEX type, so runtime is a simple check.
static bool isRawHexType(uint8_t ptype) {
    return ptype == T_HEX;
}

static float ssmDecodeSlot(const uint8_t *data, uint8_t dataOff,
                             uint8_t len, uint8_t ptype, uint8_t endian,
                             uint8_t bitNum, const char *expr) {
    if (ptype == T_NONE || ptype == T_CALC) return 0.0f;

    // Assemble raw bytes respecting endian
    uint8_t raw[4] = {0};
    if (len > 4) len = 4;
    if (endian == END_BIG) {
        for (uint8_t i = 0; i < len; i++) raw[i] = data[dataOff + i];
    } else {
        // little-endian: reverse into raw[]
        for (uint8_t i = 0; i < len; i++) raw[len - 1 - i] = data[dataOff + i];
    }

    if (ptype == T_FLOAT) {
        // SH7058S sends big-endian IEEE 754. ARM Cortex-M4 is little-endian.
        // raw[] was filled MSB-first (END_BIG) or already reversed (END_LITTLE).
        // For END_BIG:  raw[0]=MSB → need to reverse for LE memcpy.
        // For END_LITTLE: raw[0..3] is already in LE order from the reversal above.
        float f;
        if (endian == END_BIG) {
            uint8_t fb[4] = {raw[3], raw[2], raw[1], raw[0]};
            memcpy(&f, fb, 4);
        } else {
            memcpy(&f, raw, 4);
        }
        return evalExpr(expr, f);
    }

    // Assemble big-endian raw[] into a uint32 (shared by integer types and T_BIT).
    // Guard len=0: shifting a 32-bit value by 32 is undefined behaviour in C/C++.
    if (len == 0) return evalExpr(expr, 0.0f);
    uint32_t u = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16)
               | ((uint32_t)raw[2] <<  8) |  (uint32_t)raw[3];
    // Right-shift for actual size (we always put MSB at raw[0])
    u >>= (4 - len) * 8;

    if (ptype == T_BIT) {
        // Extract bit `bitNum` (0..31) from the assembled value.
        // SSM bits: len=1, bitNum 0-7 (single byte). OBD bits: len up to 4,
        // bitNum up to 31 (e.g. uint32 PID, bit=31 = the MSB).
        float x = (bitNum < 32) ? (float)((u >> bitNum) & 0x01u) : 0.0f;
        return evalExpr(expr, x);
    }

    float x;
    switch (ptype) {
        case T_I8:  x = (float)(int8_t) (u & 0xFF);       break;
        case T_I16: x = (float)(int16_t)(u & 0xFFFF);     break;
        case T_I32: x = (float)(int32_t) u;               break;
        default:    x = (float)u;                          break;  // U8 / U16 / U32
    }
    return evalExpr(expr, x);
}

// ---- OBD / ISO 15765 CAN functions ----
// Hardware: TJA1050 on D4 (CANRX) / D5 (CANTX), 500kbps.
// OBD request: CAN ID 0x7E0, data = [0x02, mode, PID, 0x55 × 5]
// OBD response: CAN ID 0x7E8, data = [len, mode+0x40, PID, A, B, C, D]
// 'A'=data[3], 'B'=data[4], 'C'=data[5], 'D'=data[6] match formula variables.

// Initialise CAN FD peripheral at 500 kbps (standard OBD2 rate).
// Returns true if hardware responds.
static bool obdCanInit() {
    return (CAN.begin(CanBitRate::BR_500k) == 1);
}

// Send a mode-01 OBD PID request to 0x7E0.
static void obdSendRequest(uint8_t pid) {
    uint8_t d[8] = {0x02, 0x01, pid, 0x55, 0x55, 0x55, 0x55, 0x55};
    CAN.write(CanMsg(CanStandardId(0x7E0), 8, d));
}

// Wait up to timeoutMs for a mode-01 response matching pid from ID 0x7E8.
// Writes A/B/C/D (up to 4 data bytes) into out[0..3]. Returns true on success.
static bool obdReadResponse(uint8_t pid, uint8_t *out, uint32_t timeoutMs) {
    memset(out, 0, 4);
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (!CAN.available()) continue;
        CanMsg msg = CAN.read();
        // Expect response from ECU (0x7E8), service 01 response (0x41), matching PID
        if ((uint32_t)msg.id == 0x7E8 && msg.data_length >= 3 &&
            msg.data[1] == 0x41 && msg.data[2] == pid) {
            for (uint8_t i = 0; i < 4 && (i + 3) < msg.data_length; i++)
                out[i] = msg.data[3 + i];
            return true;
        }
        // Ignore other CAN traffic (TPMS, body modules, etc.) and keep waiting
    }
    return false;
}

// Probe the OBD ECU: send PID 0x00 (supported PIDs), wait for any response.
// Sets g_romId to "OBD_GENERIC" on success.
static bool obdProbe() {
    obdSendRequest(0x00);
    uint8_t resp[4];
    if (!obdReadResponse(0x00, resp, PHASE1_RETRY_MS)) return false;
    strncpy(g_romId, "OBD_GENERIC", sizeof(g_romId) - 1);
    g_romId[sizeof(g_romId) - 1] = '\0';
    g_ecuInitLen = 0;   // OBD capability checking not yet implemented; skip filtering
    return true;
}

// ============================================================
// === EXPRESSION EVALUATOR ===
// ============================================================
// Runtime recursive descent parser for RomRaider formula strings.
// Handles every expr form in the RomRaider SSM + OBD definitions:
//   float literals, 'x' (SSM value), A/B/C/D (OBD raw bytes),
//   param refs P8 / [P7:psi] (units qualifier ignored), operators
//   + - * / %, unary - and !, comparisons > < >= <= == !=,
//   parentheses, and the if(cond, whenTrue, whenFalse) function.
//
// Grammar (precedence low→high):
//   expr    → sum  ( cmp_op sum )*          cmp yields 1.0 / 0.0
//   sum     → term (('+' | '-') term)*
//   term    → factor (('*' | '/' | '%') factor)*
//   factor  → ('-' | '!') factor | primary
//   primary → '(' expr ')' | if(expr,expr,expr) | NAME(...) (skipped)
//           | number | 'x' | A|B|C|D | '[' PARAM_ID ':units ]' | PARAM_ID
//
// XML entities (&gt; &lt; &amp;) are decoded to > < & when the cache
// is written, so the evaluator only sees plain operators.
// BitWise() (DS2/NCS only — unsupported protocols) parses as an
// unknown function → skipped, returns 0.
// ============================================================

static const char *g_ep;       // parse pointer into expr string
static float       g_ex;       // value of 'x' for current evaluation (SSM assembled int/float)
static float       g_eA, g_eB, g_eC, g_eD;  // OBD raw bytes A/B/C/D
static bool        g_divZero;  // set true by epTerm when divisor == 0

// Skip whitespace
static void epSkip() { while (*g_ep == ' ' || *g_ep == '\t') g_ep++; }

// Forward declarations
static float epExpr();    // top level = comparison
static float epSum();     // + -
static float epTerm();    // * / %
static float epFactor();  // unary -/!, primary

// Consume one expected character (',' or ')') with surrounding whitespace.
static void epExpect(char c) { epSkip(); if (*g_ep == c) g_ep++; }

// Look up a param ref by id → its last computed value (0 if not present).
static float epRefValue(const char *id) {
    for (uint8_t i = 0; i < g_numSel; i++)
        if (strcmp(g_sel[i].id, id) == 0) return g_sel[i].lastVal;
    return 0.0f;
}

// if(cond, whenTrue, whenFalse) — lazy: only the TAKEN branch's divide-by-zero
// counts (so the untaken branch can't raise a spurious DIV_ZERO event).
// g_ep is positioned just after the opening '('.
static float epIf() {
    float cond = epExpr();
    epExpect(',');
    bool condDZ = g_divZero;
    g_divZero = false;
    float a = epExpr();              // whenTrue
    bool aDZ = g_divZero;
    epExpect(',');
    g_divZero = false;
    float b = epExpr();              // whenFalse
    bool bDZ = g_divZero;
    epExpect(')');
    if (cond != 0.0f) { g_divZero = condDZ || aDZ; return a; }
    else              { g_divZero = condDZ || bDZ; return b; }
}

// expr → sum ( cmp_op sum )*   — comparison operators yield 1.0 / 0.0
static float epExpr() {
    float v = epSum();
    for (;;) {
        epSkip();
        if      (g_ep[0]=='>' && g_ep[1]=='=') { g_ep+=2; v = (v >= epSum()) ? 1.0f : 0.0f; }
        else if (g_ep[0]=='<' && g_ep[1]=='=') { g_ep+=2; v = (v <= epSum()) ? 1.0f : 0.0f; }
        else if (g_ep[0]=='=' && g_ep[1]=='=') { g_ep+=2; v = (v == epSum()) ? 1.0f : 0.0f; }
        else if (g_ep[0]=='!' && g_ep[1]=='=') { g_ep+=2; v = (v != epSum()) ? 1.0f : 0.0f; }
        else if (*g_ep=='>')                   { g_ep++;  v = (v >  epSum()) ? 1.0f : 0.0f; }
        else if (*g_ep=='<')                   { g_ep++;  v = (v <  epSum()) ? 1.0f : 0.0f; }
        else break;
    }
    return v;
}

// sum → term (('+' | '-') term)*
static float epSum() {
    float v = epTerm();
    for (;;) {
        epSkip();
        if      (*g_ep == '+') { g_ep++; v += epTerm(); }
        else if (*g_ep == '-') { g_ep++; v -= epTerm(); }
        else break;
    }
    return v;
}

// term → factor (('*' | '/' | '%') factor)*
static float epTerm() {
    float v = epFactor();
    for (;;) {
        epSkip();
        if (*g_ep == '*') { g_ep++; v *= epFactor(); }
        else if (*g_ep == '/') {
            g_ep++;
            float d = epFactor();
            if (d == 0.0f) { g_divZero = true; v = 0.0f; }
            else            v /= d;
        }
        else if (*g_ep == '%') {
            g_ep++;
            float d = epFactor();
            if (d == 0.0f) { g_divZero = true; v = 0.0f; }
            else            v = fmodf(v, d);
        }
        else break;
    }
    return v;
}

// factor → ('-' | '!') factor | primary
static float epFactor() {
    epSkip();
    // Unary minus / logical NOT
    if (*g_ep == '-') { g_ep++; return -epFactor(); }
    if (*g_ep == '!') { g_ep++; return (epFactor() == 0.0f) ? 1.0f : 0.0f; }
    // Parentheses
    if (*g_ep == '(') {
        g_ep++;
        float v = epExpr();
        epExpect(')');
        return v;
    }
    // Units-qualified param ref: [Pxx:units] → value of Pxx (units qualifier ignored —
    // value used as-stored; true multi-unit conversion is future work).
    if (*g_ep == '[') {
        g_ep++;
        char id[MAX_ID_LEN]; uint8_t n = 0;
        while (n < MAX_ID_LEN - 1 &&
               ((*g_ep>='A'&&*g_ep<='Z')||(*g_ep>='a'&&*g_ep<='z')||
                (*g_ep>='0'&&*g_ep<='9')||*g_ep=='_'))
            id[n++] = *g_ep++;
        id[n] = '\0';
        while (*g_ep && *g_ep != ']') g_ep++;   // skip ":units"
        if (*g_ep == ']') g_ep++;
        return epRefValue(id);
    }
    // 'x' variable (standalone in RomRaider exprs; checked before identifier reader)
    if (*g_ep == 'x') { g_ep++; return g_ex; }
    // Numeric literal (digits, dot, optional leading dot)
    if ((*g_ep >= '0' && *g_ep <= '9') || *g_ep == '.') {
        char *end;
        float v = strtof(g_ep, &end);
        g_ep = end;
        return v;
    }
    // Identifier: function call, OBD byte variable, or param ref
    if ((*g_ep >= 'A' && *g_ep <= 'Z') || (*g_ep >= 'a' && *g_ep <= 'z')) {
        char id[MAX_ID_LEN];
        uint8_t n = 0;
        while (n < MAX_ID_LEN - 1 &&
               ((*g_ep >= 'A' && *g_ep <= 'Z') ||
                (*g_ep >= 'a' && *g_ep <= 'z') ||
                (*g_ep >= '0' && *g_ep <= '9') ||
                 *g_ep == '_')) {
            id[n++] = *g_ep++;
        }
        id[n] = '\0';
        epSkip();
        // Function call?  NAME( ... )
        if (*g_ep == '(') {
            g_ep++;
            if (strcmp(id, "if") == 0) return epIf();
            // Unknown function (e.g. BitWise — DS2/NCS only): skip balanced args → 0
            int depth = 1;
            while (*g_ep && depth) {
                if (*g_ep == '(') depth++;
                else if (*g_ep == ')') depth--;
                g_ep++;
            }
            return 0.0f;
        }
        // Single-letter A/B/C/D → OBD raw byte variables
        if (n == 1) {
            if (id[0] == 'A') return g_eA;
            if (id[0] == 'B') return g_eB;
            if (id[0] == 'C') return g_eC;
            if (id[0] == 'D') return g_eD;
        }
        // Multi-char ID → param dep reference
        return epRefValue(id);
    }
    // Unrecognised character — return 0 and advance to avoid infinite loop
    if (*g_ep) g_ep++;
    return 0.0f;
}

// Evaluate expr with x=xVal (SSM params).  A/B/C/D are zeroed — unused in SSM exprs.
float evalExpr(const char *expr, float xVal) {
    g_ep = expr; g_ex = xVal;
    g_eA = g_eB = g_eC = g_eD = 0.0f;
    g_divZero = false;
    return epExpr();
}

// Evaluate a T_CALC expression — dep param IDs resolved via g_sel[].lastVal.
// Sets *divZeroOut = true if any division by zero occurred.
float evalCalcExpr(const char *expr, bool *divZeroOut) {
    g_ep = expr; g_ex = 0.0f;
    g_eA = g_eB = g_eC = g_eD = 0.0f;
    g_divZero = false;
    float result = epExpr();
    *divZeroOut = g_divZero;
    return result;
}

// Evaluate an OBD expr using raw bytes A/B/C/D from the CAN response.
// NOTE: the RomRaider v370 definitions use 'x' (assembled value), NOT A/B/C/D,
// so the OBD streaming path uses ssmDecodeSlot() instead and this function is
// currently unused. Kept for definition files that DO use the A/B/C/D convention.
float evalExprOBD(const char *expr, uint8_t A, uint8_t B, uint8_t C, uint8_t D) {
    g_ep = expr; g_ex = 0.0f;
    g_eA = (float)A; g_eB = (float)B; g_eC = (float)C; g_eD = (float)D;
    g_divZero = false;
    return epExpr();
}

// ============================================================
// === XML PARSER ===
// ============================================================
// Two parsers:
//   findDefsFile()  — scan logger/ for first logger_*.xml file
//   parseProfile()  — read logger.xml; extract protocol= and
//                     selected param IDs + units strings
//
// The heavy definitions scanner (parseDefinitions) is in
// ADDRESS MAP because it writes directly to address_map.csv.
// ============================================================

// Temporary storage for units strings during cache generation.
// parseProfile() writes here; parseDefinitions() reads here.
// Index matches g_sel[i].
static char g_parseUnits[MAX_SELECTED][32];

// ---- Attribute extraction ----

// Find attrEq (e.g. "id=") in line, copy the quoted value into out[0..maxLen-1].
// Handles both single and double quotes.  Returns true on success.
static bool extractAttr(const char *line, const char *attrEq,
                        char *out, uint8_t maxLen) {
    const char *p = strstr(line, attrEq);
    if (!p) return false;
    p += strlen(attrEq);
    if (*p != '"' && *p != '\'') return false;
    char q = *p++;
    uint8_t n = 0;
    while (*p && *p != q && n < (uint8_t)(maxLen - 1)) out[n++] = *p++;
    out[n] = '\0';
    return true;
}

// Read text content of a self-contained element, e.g. "<address>0xFF52B4</address>".
// Finds '>' then reads until '<'.
static void extractContent(const char *line, char *out, uint8_t maxLen) {
    const char *p = strchr(line, '>');
    if (!p) { out[0] = '\0'; return; }
    p++;
    uint8_t n = 0;
    while (*p && *p != '<' && n < (uint8_t)(maxLen - 1)) out[n++] = *p++;
    out[n] = '\0';
}

// Trim leading/trailing spaces and tabs from s, in place.
// Applied to extracted expr strings so e.g. expr=" x " still matches the
// raw-hex identity check and the evaluator gets a clean formula.
static void trimWs(char *s) {
    char *p = s;
    while (*p == ' ' || *p == '\t') p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n && (s[n-1] == ' ' || s[n-1] == '\t')) s[--n] = '\0';
}

// ---- Directory scan ----

// Find the first file in /logger/ whose name starts with "logger_" and ends
// with ".xml".  Writes result into g_defsFilename[].
// Returns true if found.
bool findDefsFile() {
    SdFile dir, entry;
    char fname[48];
    bool found = false;

    if (!dir.open("/logger")) return false;

    while (!found && entry.openNext(&dir, O_READ)) {
        if (!entry.isDir()) {
            entry.getName(fname, sizeof(fname));
            uint8_t flen = strlen(fname);
            if (flen > 11 &&
                strncmp(fname, "logger_", 7) == 0 &&
                strcmp(fname + flen - 4, ".xml") == 0) {
                strncpy(g_defsFilename, fname, sizeof(g_defsFilename) - 1);
                g_defsFilename[sizeof(g_defsFilename) - 1] = '\0';
                found = true;
            }
        }
        entry.close();
    }
    dir.close();
    return found;
}

// ---- Profile parser ----

// Parse logger/logger.xml.
// Fills g_sel[0..g_numSel-1].id and g_parseUnits[0..g_numSel-1] with units.
// Sets g_protocol.
// Returns true on success; false if file missing, no params selected, or
// protocol not supported (writes status.log entry before returning false).
// Strip XML comment regions (<!-- ... -->) from a line IN PLACE, carrying the
// open/closed state across lines via inComment. Needed because generated profiles
// embed instruction text (which literally contains livedata="selected") and inline
// <!-- name --> comments that must NOT be parsed as real selections.
static void stripXmlComments(char *s, bool &inComment) {
    char *out = s, *p = s;
    while (*p) {
        if (inComment) {
            char *end = strstr(p, "-->");
            if (!end) { *out = '\0'; return; }
            p = end + 3;
            inComment = false;
        } else {
            char *start = strstr(p, "<!--");
            if (!start) { while (*p) *out++ = *p++; break; }
            while (p < start) *out++ = *p++;
            p = start + 4;
            inComment = true;
        }
    }
    *out = '\0';
}

bool parseProfile(const char *path) {
    SdFile f;
    char line[LINE_BUF_SIZE];

    if (!f.open(path, O_READ)) return false;

    g_numSel = 0;
    g_protocol = PROTO_SSM;   // default

    bool inComment = false;
    while (f.fgets(line, LINE_BUF_SIZE) > 0) {
        stripXmlComments(line, inComment);   // remove commented regions first
        // Extract protocol from <profile protocol="SSM">
        if (strstr(line, "<profile") && strstr(line, "protocol=")) {
            char proto[16];
            if (extractAttr(line, "protocol=", proto, sizeof(proto))) {
                if (strcmp(proto, "OBD") == 0)       g_protocol = PROTO_OBD;
                else if (strcmp(proto, "SSM") == 0)  g_protocol = PROTO_SSM;
                // DS2 / NCS → also unsupported; detected below
            }
        }

        // Selected parameter or switch: anything with livedata="selected"
        // Covers <parameter id="P8" livedata="selected" units="rpm"/>
        // and   <switch id="S13" livedata="selected"/>  (switches have no units)
        if (strstr(line, "livedata=\"selected\"")) {
            if (g_numSel >= MAX_SELECTED) continue;

            char pid[MAX_ID_LEN], units[32];
            if (!extractAttr(line, "id=", pid, sizeof(pid))) continue;

            strncpy(g_sel[g_numSel].id, pid, MAX_ID_LEN - 1);
            g_sel[g_numSel].id[MAX_ID_LEN - 1] = '\0';

            if (extractAttr(line, "units=", units, sizeof(units))) {
                strncpy(g_parseUnits[g_numSel], units, 31);
                g_parseUnits[g_numSel][31] = '\0';
            } else {
                g_parseUnits[g_numSel][0] = '\0';
            }

            // Zero-init runtime fields
            g_sel[g_numSel].address      = 0;
            g_sel[g_numSel].len          = 0;
            g_sel[g_numSel].ptype        = T_NONE;
            g_sel[g_numSel].endian       = END_BIG;
            g_sel[g_numSel].bitNum       = 0;
            g_sel[g_numSel].expr[0]      = 'x';
            g_sel[g_numSel].expr[1]      = '\0';
            g_sel[g_numSel].fetchIdx     = FETCH_NONE;
            g_sel[g_numSel].lastVal      = 0.0f;
            g_sel[g_numSel].divZeroLogged= false;
            g_sel[g_numSel].hidden       = false;
            g_sel[g_numSel].name[0]      = '\0';

            g_numSel++;
        }
    }
    f.close();
    return (g_numSel > 0);
}

// ============================================================
// === ADDRESS MAP ===
// ============================================================
// Builds and reads address_map.csv — the COMPLETE per-ECU param dictionary.
// Holds EVERY param the ECU supports, ONE ROW PER CONVERSION (units variant). The
// units column lets the loader pick the row matching logger.xml, so changing which
// params (or which units) you log never needs a regen — only ECU/DEFS change does.
//
// Cache format (address_map.csv):
//   DEBUG=false                          <- user-editable verbose-debug toggle
//   ECU=2F12515506
//   DEFS=logger_IMP_EN_v370.xml
//   param_id,name,address,len,type,endian,expr,units,deps
//   P7,Manifold Pressure,0x00000D,1,u8,big,x/100,bar,
//   P7,Manifold Pressure,0x00000D,1,u8,big,x*37/255,psi,
//   E83,A/F Learning #1,0xFF52B4,2,u16,big,(x*.01220703)-100,%,
//   P200,Engine Load (Calc),CALCULATED,0,calculated,big,(P12*60)/P8,g/rev,P8:P12
//   E_IMMO_EAA_EAD,Auth Flags,0xFF5EAA,4,hex,big,x,raw,
// ============================================================

// ---- Conversion helpers ----

// Convert storagetype string + length to a T_* constant.
static uint8_t stToType(const char *st, uint8_t len) {
    if (strcmp(st, "float")  == 0) return T_FLOAT;
    if (strcmp(st, "int8")   == 0) return T_I8;
    if (strcmp(st, "int16")  == 0) return T_I16;
    if (strcmp(st, "int32")  == 0) return T_I32;
    if (strcmp(st, "uint8")  == 0) return T_U8;
    if (strcmp(st, "uint16") == 0) return T_U16;
    if (strcmp(st, "uint32") == 0) return T_U32;
    // Absent storagetype — infer from length
    if (len == 1) return T_U8;
    if (len == 2) return T_U16;
    return T_U32;
}

// Byte width implied by a T_* type. Used to derive `len` for OBD params, which
// carry no length= attribute (their byte count comes from storagetype).
// NOTE: 3-byte (u24) params can only come from an explicit length="3" (SSM
// E_IMMO_*), never from a storagetype — so this never needs to return 3.
static uint8_t typeWidth(uint8_t pt) {
    switch (pt) {
        case T_U16: case T_I16:            return 2;
        case T_U32: case T_I32: case T_FLOAT: return 4;
        default:                           return 1;  // U8/I8/BIT
    }
}

// Convert T_* + bitNum to cache column string.
// T_BIT encodes the bit index (0..31) as "bitN" — supports OBD multi-byte bit
// params (e.g. bit=31 on a uint32 PID), not just single-byte SSM bits (0..7).
static const char *typeToStr(uint8_t ptype, uint8_t bitNum) {
    static char bitbuf[8];
    switch (ptype) {
        case T_U8:   return "u8";
        case T_U16:  return "u16";
        case T_U32:  return "u32";
        case T_I8:   return "i8";
        case T_I16:  return "i16";
        case T_I32:  return "i32";
        case T_FLOAT:return "float";
        case T_CALC: return "calculated";
        case T_NONE: return "notfound";
        case T_HEX:  return "hex";
        case T_BIT:
            snprintf(bitbuf, sizeof(bitbuf), "bit%u", bitNum);
            return bitbuf;   // consumed immediately by the caller's snprintf
    }
    return "u8";
}

// Convert cache type string back to T_* constant; writes bit number (0..31) to *bitOut.
static uint8_t strToType(const char *s, uint8_t *bitOut) {
    *bitOut = 0;
    if (strcmp(s,"u8")   == 0) return T_U8;
    if (strcmp(s,"u16")  == 0) return T_U16;
    if (strcmp(s,"u32")  == 0) return T_U32;
    if (strcmp(s,"i8")   == 0) return T_I8;
    if (strcmp(s,"i16")  == 0) return T_I16;
    if (strcmp(s,"i32")  == 0) return T_I32;
    if (strcmp(s,"float")== 0) return T_FLOAT;
    if (strcmp(s,"calculated")==0) return T_CALC;
    if (strcmp(s,"notfound")  ==0) return T_NONE;
    if (strcmp(s,"hex")       ==0) return T_HEX;
    if (strncmp(s,"bit",3)==0 && s[3]>='0' && s[3]<='9') {
        *bitOut = (uint8_t)atoi(s + 3);   // 0..31
        return T_BIT;
    }
    return T_NONE;
}

// Extract the Nth comma-delimited field from a CSV line.
static bool csvField(const char *line, uint8_t n, char *out, uint8_t maxLen) {
    const char *p = line;
    for (uint8_t i = 0; i < n; i++) {
        while (*p && *p != ',' && *p != '\n' && *p != '\r') p++;
        if (*p != ',') return false;
        p++;
    }
    uint8_t k = 0;
    while (*p && *p != ',' && *p != '\n' && *p != '\r' && k < (uint8_t)(maxLen-1))
        out[k++] = *p++;
    out[k] = '\0';
    return true;
}

// ---- ROM ID helpers ----

// True if the 10-char g_romId appears in the quoted comma-separated list on
// an <ecu id="..."> line.  Scans in-place — no copy buffer needed.
// (ecu id= lines can have 200+ chars; a fixed buffer would truncate them.)
static bool ecuIdInLine(const char *line) {
    const char *p = strstr(line, "id=");
    if (!p) return false;
    p += 3;
    if (*p != '"' && *p != '\'') return false;
    char q = *p++;
    while (*p && *p != q) {
        bool match = true;
        for (uint8_t i = 0; i < 10; i++) {
            if (!p[i] || p[i] != g_romId[i]) { match = false; break; }
        }
        if (match) {
            char nx = p[10];
            // Accept comma (next ID), or quote (end of attribute) — but NOT '\0':
            // a '\0' here means fgets truncated the line at exactly this position,
            // which would be a false match on a line longer than LINE_BUF_SIZE.
            if (nx == ',' || nx == '"' || nx == '\'') return true;
        }
        while (*p && *p != ',' && *p != q) p++;
        if (*p == ',') p++;
    }
    return false;
}

// ---- Definitions file scanner ----
// State constants for parseDefinitions()
#define DP_IDLE       0
#define DP_SSM        1
#define DP_SKIP_P     2
#define DP_SKIP_E     3
#define DP_IN_P       4
#define DP_IN_P_CONV  5
#define DP_IN_P_DEP   6
#define DP_IN_E       7
#define DP_IN_E_MATCH 8
#define DP_IN_E_SKIP  9
#define DP_IN_E_CONV  10

// Write one completed param row to the open cache file.
// Commas in pname are replaced with ';' to preserve CSV column alignment.
// The expr field is sanitised: XML entities (&gt; &lt; &amp;) are decoded to
// > < & , and commas (which appear inside if(a,b,c)) are escaped to '|' so they
// don't collide with the CSV delimiter. Cache loaders restore '|' → ',' on read.
static void writeCacheRow(SdFile &cf,
                           const char *pid, const char *pname,
                           uint32_t addr, uint8_t len,
                           uint8_t ptype, uint8_t endian, uint8_t bitNum,
                           const char *expr, const char *units, const char *deps) {
    char addrStr[12];
    if (ptype == T_CALC)      snprintf(addrStr, sizeof(addrStr), "CALCULATED");
    else if (ptype == T_NONE) snprintf(addrStr, sizeof(addrStr), "NOTFOUND");
    else                      snprintf(addrStr, sizeof(addrStr), "0x%06lX", (unsigned long)addr);

    // Sanitise name: copy and replace any commas with semicolons
    char safeName[MAX_NAME_LEN];
    uint8_t k = 0;
    for (const char *p = pname; *p && k < MAX_NAME_LEN - 1; p++, k++)
        safeName[k] = (*p == ',') ? ';' : *p;
    safeName[k] = '\0';

    // Sanitise units the same way (units strings have no commas, but be safe)
    char safeUnits[32];
    k = 0;
    for (const char *p = units; *p && k < (uint8_t)(sizeof(safeUnits)-1); p++, k++)
        safeUnits[k] = (*p == ',') ? ';' : *p;
    safeUnits[k] = '\0';

    // Sanitise expr: decode &gt; &lt; &amp; entities; escape commas to '|'.
    char safeExpr[MAX_EXPR_LEN];
    {
        const char *s = expr; uint8_t j = 0;
        while (*s && j < MAX_EXPR_LEN - 1) {
            if      (strncmp(s, "&gt;",  4) == 0) { safeExpr[j++] = '>'; s += 4; }
            else if (strncmp(s, "&lt;",  4) == 0) { safeExpr[j++] = '<'; s += 4; }
            else if (strncmp(s, "&amp;", 5) == 0) { safeExpr[j++] = '&'; s += 5; }
            else if (*s == ',')                   { safeExpr[j++] = '|'; s++;    }
            else                                  { safeExpr[j++] = *s++;        }
        }
        safeExpr[j] = '\0';
    }

    // Columns: param_id,name,address,len,type,endian,expr,units,deps
    char row[LINE_BUF_SIZE];
    snprintf(row, sizeof(row), "%s,%s,%s,%u,%s,%s,%s,%s,%s\n",
             pid, safeName, addrStr, (unsigned)len,
             typeToStr(ptype, bitNum),
             (endian == END_BIG) ? "big" : "little",
             safeExpr, safeUnits[0] ? safeUnits : "-", deps);
    cf.print(row);
}

// Parse one <conversion ...> line and write a cache row for it (one units variant).
// Used by both the P-param and E-param conversion states, so EACH conversion the ECU
// supports becomes its own row, distinguished by the units column.
static void writeConversionRow(SdFile &cf, const char *line,
                               const char *wId, const char *wName, uint32_t wAddr,
                               uint8_t wLen, uint8_t wBit, bool wHasBit, bool wHasLen,
                               bool wIsCalc, const char *wCalcDeps) {
    char cunits[32]={0}, cstoType[16]={0}, cendian[16]={0}, cexpr[MAX_EXPR_LEN]={0};
    extractAttr(line, "units=",       cunits,   sizeof(cunits));
    extractAttr(line, "storagetype=", cstoType, sizeof(cstoType));
    if (!extractAttr(line, "endian=",  cendian,  sizeof(cendian))) strcpy(cendian, "big");
    extractAttr(line, "expr=", cexpr, sizeof(cexpr));
    trimWs(cexpr);   // tolerate expr=" x " whitespace (raw-hex check + evaluator)

    uint8_t baseType = stToType(cstoType, wLen);
    uint8_t pt = wIsCalc ? T_CALC : (wHasBit ? T_BIT : baseType);
    // Raw integer register dump → T_HEX (exact hex): integer + identity expr "x"
    // + units beginning "raw" (immo registers, raw ecu values, MerpMod ids).
    if (pt <= T_I32 && cexpr[0]=='x' && cexpr[1]=='\0' && strncmp(cunits,"raw",3)==0)
        pt = T_HEX;
    uint8_t en = (strcmp(cendian,"little")==0) ? END_LITTLE : END_BIG;
    // Byte width: explicit length= (SSM) else storagetype width (OBD); bits use it too.
    uint8_t rowLen = wIsCalc ? 0 : (wHasLen ? wLen : typeWidth(baseType));
    writeCacheRow(cf, wId, wName[0]?wName:"Unknown", wAddr, rowLen, pt, en, wBit,
                  cexpr, cunits, wIsCalc ? wCalcDeps : "");
}

// Scan the definitions file and write the COMPLETE per-ECU dictionary to cacheFile:
// EVERY param the ECU supports — capability-checked P params + switches, ROM-matched
// E params + custom ecuparams — with ONE ROW PER CONVERSION (units variant). The units
// column lets loadCacheForSelected pick the row matching logger.xml, so changing which
// params (or which units) you log never triggers a regen. Unsupported params are simply
// omitted. Capability bytes must already be in g_ecuInitRaw[]/g_ecuInitLen.
static void parseDefinitions(SdFile &cacheFile) {
    char defsPath[56];
    snprintf(defsPath, sizeof(defsPath), "/logger/%s", g_defsFilename);

    SdFile df;
    if (!df.open(defsPath, O_READ)) return;

    char line[LINE_BUF_SIZE];
    uint8_t state = DP_IDLE;

    // Working param during scan
    char  wId[MAX_ID_LEN] = {0}, wName[MAX_NAME_LEN] = {0};
    uint32_t wAddr = 0;
    uint8_t  wLen = 1, wBit = 0;
    bool     wHasBit = false, wIsCalc = false, wHasLen = false;
    char     wCalcDeps[MAX_EXPR_LEN] = {0};

    // Determine which protocol section to scan
    const char *targetSection = (g_protocol == PROTO_OBD) ? "id=\"OBD\"" : "id=\"SSM\"";

    while (df.fgets(line, LINE_BUF_SIZE) > 0) {
        // ---- IDLE: wait for target protocol section ----
        if (state == DP_IDLE) {
            if (strstr(line, "<protocol") && strstr(line, targetSection))
                state = DP_SSM;
            continue;
        }

        // ---- Exit target protocol section (skip OBD/DS2/NCS) ----
        if (strstr(line, "</protocol>")) break;

        // ====================================================
        if (state == DP_SSM) {
            // ---- Switch: self-closing; capability-checked T_BIT row (no units) ----
            if (strstr(line, "<switch ") && strstr(line, "id=")) {
                char pid[MAX_ID_LEN];
                if (extractAttr(line, "id=", pid, sizeof(pid))) {
                    char swName[MAX_NAME_LEN]={0}, swByte[12]={0}, swBit[4]={0}, swBI[8]={0};
                    extractAttr(line, "name=", swName, sizeof(swName));
                    extractAttr(line, "byte=", swByte, sizeof(swByte));
                    extractAttr(line, "bit=",  swBit,  sizeof(swBit));
                    // RomRaider <switch> capability uses the SAME bit= value as ecuBit.
                    bool capChk = (g_protocol == PROTO_SSM) &&
                                  extractAttr(line, "ecubyteindex=", swBI, sizeof(swBI));
                    bool ok = !capChk || ssmParamSupported(
                                  (uint8_t)atoi(swBI), (uint8_t)atoi(swBit));
                    if (ok) {   // unsupported switches are simply omitted from the dictionary
                        uint32_t addr = (uint32_t)strtoul(swByte, NULL, 16);
                        writeCacheRow(cacheFile, pid, swName[0]?swName:"Unknown",
                                      addr, 1, T_BIT, END_BIG, (uint8_t)atoi(swBit), "x", "", "");
                    }
                }
            }
            // ---- Parameter (P): capability filter at entry; unsupported → skip ----
            else if (strstr(line, "<parameter ") && strstr(line, "id=")) {
                char pid[MAX_ID_LEN];
                if (!extractAttr(line, "id=", pid, sizeof(pid))) continue;
                char tmpBI[8], tmpBit[8];
                bool hasCap = (g_protocol == PROTO_SSM) &&
                              extractAttr(line, "ecubyteindex=", tmpBI, sizeof(tmpBI)) &&
                              extractAttr(line, "ecubit=",       tmpBit, sizeof(tmpBit));
                if (hasCap && !ssmParamSupported((uint8_t)atoi(tmpBI),(uint8_t)atoi(tmpBit))) {
                    state = DP_SKIP_P; continue;
                }
                strncpy(wId, pid, MAX_ID_LEN-1); wId[MAX_ID_LEN-1]='\0';
                wName[0]='\0'; extractAttr(line, "name=", wName, sizeof(wName));
                wAddr=0; wLen=1; wBit=0; wHasBit=false; wIsCalc=false; wHasLen=false;
                wCalcDeps[0]='\0';
                state = DP_IN_P;
            }
            // ---- Ecuparam (E): ROM match decided in DP_IN_E ----
            else if (strstr(line, "<ecuparam ") && strstr(line, "id=")) {
                char pid[MAX_ID_LEN];
                if (!extractAttr(line, "id=", pid, sizeof(pid))) continue;
                strncpy(wId, pid, MAX_ID_LEN-1); wId[MAX_ID_LEN-1]='\0';
                wName[0]='\0'; extractAttr(line, "name=", wName, sizeof(wName));
                wAddr=0; wLen=1; wBit=0; wHasBit=false; wIsCalc=false; wHasLen=false;
                state = DP_IN_E;
            }
        }

        // ====================================================
        else if (state == DP_SKIP_P) {
            if (strstr(line, "</parameter>")) state = DP_SSM;
        }
        else if (state == DP_SKIP_E) {
            if (strstr(line, "</ecuparam>")) state = DP_SSM;
        }

        // ====================================================
        else if (state == DP_IN_P) {
            if (strstr(line, "<depends")) {
                wIsCalc = true; wCalcDeps[0]='\0';
                state = DP_IN_P_DEP;
            }
            else if (strstr(line, "<address")) {
                // Extract length= and bit= attributes
                char tmp[8];
                wHasLen = extractAttr(line, "length=", tmp, sizeof(tmp));
                wLen = wHasLen ? (uint8_t)atoi(tmp) : 1;   // OBD has no length= → derive later
                wHasBit = extractAttr(line, "bit=", tmp, sizeof(tmp));
                wBit = wHasBit ? (uint8_t)atoi(tmp) : 0;
                // Extract hex address from element content
                char content[12];
                extractContent(line, content, sizeof(content));
                wAddr = (uint32_t)strtoul(content, NULL, 16);
                if (wHasBit) wLen = 1;
                state = DP_IN_P_CONV;
            }
            else if (strstr(line, "</parameter>")) state = DP_SSM;   // no addr/depends — omit
        }

        // ====================================================
        else if (state == DP_IN_P_DEP) {
            if (strstr(line, "<ref ")) {
                char refId[MAX_ID_LEN];
                if (extractAttr(line, "parameter=", refId, sizeof(refId))) {
                    if (wCalcDeps[0]) strncat(wCalcDeps, ":", MAX_EXPR_LEN-strlen(wCalcDeps)-1);
                    strncat(wCalcDeps, refId, MAX_EXPR_LEN-strlen(wCalcDeps)-1);
                }
            }
            else if (strstr(line, "</depends>")) state = DP_IN_P_CONV;
        }

        // ==================================================== P conversions
        // Write one row per <conversion> (each units variant) — the units column
        // lets the loader pick the right one. T_HEX / width logic in writeConversionRow.
        else if (state == DP_IN_P_CONV) {
            if (strstr(line, "<conversion"))
                writeConversionRow(cacheFile, line, wId, wName, wAddr, wLen, wBit,
                                   wHasBit, wHasLen, wIsCalc, wCalcDeps);
            else if (strstr(line, "</parameter>")) state = DP_SSM;
        }

        // ====================================================
        else if (state == DP_IN_E) {
            if (strstr(line, "<ecu ") || strstr(line, "<ecu\t")) {
                if (ecuIdInLine(line))
                    state = DP_IN_E_MATCH;
                else
                    state = DP_IN_E_SKIP;
            }
            else if (strstr(line, "</ecuparam>")) state = DP_SSM;   // no ROM match — omit
        }

        // ====================================================
        else if (state == DP_IN_E_MATCH) {
            if (strstr(line, "<address")) {
                char tmp[8]; char content[12];
                wHasLen = extractAttr(line, "length=", tmp, sizeof(tmp));
                wLen = wHasLen ? (uint8_t)atoi(tmp) : 1;
                wHasBit = extractAttr(line, "bit=", tmp, sizeof(tmp));
                wBit = wHasBit ? (uint8_t)atoi(tmp) : 0;
                if (wHasBit) wLen = 1;
                extractContent(line, content, sizeof(content));
                wAddr = (uint32_t)strtoul(content, NULL, 16);
                state = DP_IN_E_CONV;
            }
            else if (strstr(line, "</ecu>")) state = DP_IN_E;
        }

        // ====================================================
        else if (state == DP_IN_E_SKIP) {
            if (strstr(line, "</ecu>")) state = DP_IN_E;
        }

        // ==================================================== E conversions
        // Write one row per <conversion> (each units variant) at the ROM-matched address.
        else if (state == DP_IN_E_CONV) {
            if (strstr(line, "<conversion"))
                writeConversionRow(cacheFile, line, wId, wName, wAddr, wLen, wBit,
                                   wHasBit, wHasLen, false, "");
            else if (strstr(line, "</ecuparam>")) state = DP_SSM;
        }
        // (Complete dictionary: scan the whole SSM section — no early exit.)
    } // while fgets

    df.close();
}

// ---- Cache file I/O ----

// Forward declarations for functions defined later (used by the loaders below).
void writeStatusLog(const char *event, const char *detail);  // SD OPS
DateTime queryRTC();                                          // RTC OPS
bool regenCache();                                           // SD OPS

// Read address_map.csv header lines.
// Header order: DEBUG= , ECU= , DEFS= , then the column header.
// Returns: 0 = file missing/old format, 1 = ROM ID mismatch, 2 = DEFS mismatch, 3 = OK.
// Side effect: sets g_verboseDebug from the DEBUG= line.
uint8_t checkCacheHeaders() {
    SdFile f;
    if (!f.open(ADDR_MAP_FILE, O_READ)) return 0;

    char line[80];
    // Line 1: DEBUG=true|false  (old-format caches lack this → treated as missing → regen)
    if (f.fgets(line, sizeof(line)) <= 0) { f.close(); return 0; }
    line[strcspn(line, "\r\n")] = '\0';
    if (strncmp(line, "DEBUG=", 6) != 0) { f.close(); return 0; }
    g_verboseDebug = (strcmp(line + 6, "true") == 0 || line[6] == '1');

    // Line 2: ECU=XXXXXXXXXX
    if (f.fgets(line, sizeof(line)) <= 0) { f.close(); return 0; }
    line[strcspn(line, "\r\n")] = '\0';
    char romFromFile[11];
    if (strncmp(line, "ECU=", 4) != 0) { f.close(); return 0; }
    strncpy(romFromFile, line+4, 10); romFromFile[10]='\0';
    if (strcmp(romFromFile, g_romId) != 0) { f.close(); return 1; }

    // Line 3: DEFS=filename
    if (f.fgets(line, sizeof(line)) <= 0) { f.close(); return 0; }
    line[strcspn(line, "\r\n")] = '\0';
    if (strncmp(line, "DEFS=", 5) != 0) { f.close(); return 0; }
    if (strcmp(line+5, g_defsFilename) != 0) { f.close(); return 2; }

    // Line 4: column header — validates the cache FORMAT. An old-format cache (no
    // units column) has the same DEBUG/ECU/DEFS but a different column header, so
    // this check forces a regen to the new format.
    if (f.fgets(line, sizeof(line)) <= 0) { f.close(); return 0; }
    line[strcspn(line, "\r\n")] = '\0';
    if (strcmp(line, "param_id,name,address,len,type,endian,expr,units,deps") != 0) {
        f.close(); return 0;   // wrong/old format → treat as missing → regen
    }

    f.close();
    return 3;
}

// Load selected params from the COMPLETE-dictionary cache into g_sel[].
// Matches each selected param by id AND units (the units column). units="" in
// logger.xml (e.g. switches) takes the first row for that id. A selected (id,units)
// that isn't in the dictionary means the ECU doesn't support it → marked NOTFOUND
// (logs 0.00); this NEVER triggers a regen (the dictionary is authoritative).
// Returns false only on a structurally bad cache (header read failure) → regen.
bool loadCacheForSelected() {
    for (uint8_t si = 0; si < g_numSel; si++) {
        SdFile f;
        if (!f.open(ADDR_MAP_FILE, O_READ)) return false;

        char line[LINE_BUF_SIZE];
        bool hit = false;
        // Skip 4 header lines (DEBUG=, ECU=, DEFS=, column header)
        bool headersOk = true;
        for (uint8_t h = 0; h < 4; h++) {
            if (f.fgets(line, LINE_BUF_SIZE) <= 0) { headersOk = false; break; }
        }
        if (!headersOk) { f.close(); return false; }

        uint8_t idLen = strlen(g_sel[si].id);
        bool wantAnyUnits = (g_parseUnits[si][0] == '\0');

        while (f.fgets(line, LINE_BUF_SIZE) > 0) {
            if (strncmp(line, g_sel[si].id, idLen) != 0 || line[idLen] != ',') continue;

            // Fields: 0=id,1=name,2=address,3=len,4=type,5=endian,6=expr,7=units,8=deps
            char f1[MAX_NAME_LEN], f2[12], f3[4], f4[16], f5[8], f6[MAX_EXPR_LEN], f7[32];
            csvField(line, 1, f1, sizeof(f1));   // name
            csvField(line, 2, f2, sizeof(f2));   // address
            csvField(line, 3, f3, sizeof(f3));   // len
            csvField(line, 4, f4, sizeof(f4));   // type
            csvField(line, 5, f5, sizeof(f5));   // endian
            csvField(line, 6, f6, sizeof(f6));   // expr
            csvField(line, 7, f7, sizeof(f7));   // units

            // Require the units to match the logger.xml choice (or take first if none).
            if (!wantAnyUnits && strcmp(f7, g_parseUnits[si]) != 0) continue;

            strncpy(g_sel[si].name, f1, MAX_NAME_LEN-1);
            g_sel[si].name[MAX_NAME_LEN-1] = '\0';

            uint8_t bn = 0;
            g_sel[si].ptype  = strToType(f4, &bn);
            g_sel[si].bitNum = bn;
            g_sel[si].endian = (strcmp(f5, "little")==0) ? END_LITTLE : END_BIG;
            strncpy(g_sel[si].expr, f6, MAX_EXPR_LEN-1);
            g_sel[si].expr[MAX_EXPR_LEN-1] = '\0';
            // Restore escaped commas (if(a,b,c) args were stored as '|')
            for (char *e = g_sel[si].expr; *e; e++) if (*e == '|') *e = ',';

            if (strcmp(f2, "NOTFOUND") == 0 || strcmp(f2, "CALCULATED") == 0) {
                g_sel[si].address = 0;
                g_sel[si].len     = 0;
                // ptype already set (T_NONE or T_CALC)
            } else {
                g_sel[si].address = (uint32_t)strtoul(f2, NULL, 16);
                g_sel[si].len     = (uint8_t)atoi(f3);
            }

            g_sel[si].fetchIdx      = FETCH_NONE;
            g_sel[si].lastVal       = 0.0f;
            g_sel[si].divZeroLogged = false;
            g_sel[si].hidden        = false;
            hit = true;
            break;
        }
        f.close();

        if (!hit) {
            // (id,units) not in the dictionary → ECU doesn't support it. Mark NOTFOUND
            // (log 0.00). NO regen — the complete dictionary already has everything.
            g_sel[si].ptype   = T_NONE;
            g_sel[si].address = 0; g_sel[si].len = 0; g_sel[si].bitNum = 0;
            g_sel[si].endian  = END_BIG;
            g_sel[si].expr[0] = 'x'; g_sel[si].expr[1] = '\0';
            g_sel[si].fetchIdx = FETCH_NONE;
            g_sel[si].lastVal = 0.0f; g_sel[si].divZeroLogged = false;
            g_sel[si].hidden  = false;
            if (g_sel[si].name[0] == '\0') {
                strncpy(g_sel[si].name, "Unknown", MAX_NAME_LEN-1);
                g_sel[si].name[MAX_NAME_LEN-1] = '\0';
            }
            char det[MAX_ID_LEN + 44];
            snprintf(det, sizeof(det), "%s not supported by ECU — logging zero", g_sel[si].id);
            writeStatusLog("NOTFOUND", det);
        }
    }
    return true;
}

// ---- Auto-generated profiles (unknown / profile-less ECU) ----

// The 12 basic P-params enabled by default in any generated profile.
static const char *BASIC_PARAMS[] = {
    "P8","P2","P9","P7","P11","P10","P12","P13","P3","P4","P14","P17"
};
static bool isBasicParam(const char *id) {
    for (uint8_t i = 0; i < 12; i++)
        if (strcmp(BASIC_PARAMS[i], id) == 0) return true;
    return false;
}

// True if the cache holds any E-prefix (ecuparam) row → the ECU's ROM matched the
// definitions → "known" ECU. Selection-independent (doesn't depend on logger.xml).
static bool cacheHasEParams() {
    SdFile f;
    if (!f.open(ADDR_MAP_FILE, O_READ)) return false;
    char line[LINE_BUF_SIZE];
    for (uint8_t h = 0; h < 4; h++) { if (f.fgets(line, sizeof(line)) <= 0) { f.close(); return false; } }
    bool hasE = false;
    while (f.fgets(line, sizeof(line)) > 0) { if (line[0] == 'E') { hasE = true; break; } }
    f.close();
    return hasE;
}

// Emit one profile section by streaming the cache (dedup consecutive rows by id,
// using each param's FIRST/default conversion). mode: 'i'=comment index,
// 'p'=<parameter> lines (non-switch ids), 's'=<switch> lines (S-prefix ids).
static void emitProfileSection(SdFile &pf, char mode) {
    SdFile f;
    if (!f.open(ADDR_MAP_FILE, O_READ)) return;
    char line[LINE_BUF_SIZE];
    for (uint8_t h = 0; h < 4; h++) f.fgets(line, sizeof(line));   // skip headers
    char lastId[MAX_ID_LEN] = {0};
    char id[MAX_ID_LEN], name[MAX_NAME_LEN], units[32], buf[LINE_BUF_SIZE];
    while (f.fgets(line, sizeof(line)) > 0) {
        if (!csvField(line, 0, id, sizeof(id))) continue;
        if (id[0] == '\0' || strcmp(id, lastId) == 0) continue;   // dedup: first row per id
        strncpy(lastId, id, sizeof(lastId)-1); lastId[sizeof(lastId)-1] = '\0';
        char type[16];
        csvField(line, 4, type, sizeof(type));
        if (strcmp(type, "calculated") == 0) continue;   // exclude CALC params (v1)
        csvField(line, 1, name,  sizeof(name));
        csvField(line, 7, units, sizeof(units));
        bool isSw  = (id[0] == 'S');
        bool basic = isBasicParam(id);
        if (mode == 'i') {
            snprintf(buf, sizeof(buf), "    %c %-7s %s\n", basic ? '*' : ' ', id, name);
            pf.print(buf);
        } else if (mode == 'p' && !isSw) {
            snprintf(buf, sizeof(buf),
                     "        <parameter id=\"%s\" %sunits=\"%s\"/>   <!-- %s -->\n",
                     id, basic ? "livedata=\"selected\" " : "", units, name);
            pf.print(buf);
        } else if (mode == 's' && isSw) {
            snprintf(buf, sizeof(buf),
                     "        <switch id=\"%s\"%s/>   <!-- %s -->\n",
                     id, basic ? " livedata=\"selected\"" : "", name);
            pf.print(buf);
        }
    }
    f.close();
}

// Write a complete logger profile (logger.xml or unknownecu.xml) FROM the cache:
// lists every supported param (P + S, plus E if the ECU is known — they're in the
// cache), 12 basic P-params enabled, the rest disabled, with name comments and a
// RomRaider-safe header comment block. isUnknown only tweaks the header wording.
static bool generateProfile(const char *path, bool isUnknown) {
    SdFile pf;
    if (!pf.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return false;

    DateTime now = queryRTC();
    char hdr[96];

    pf.print(F("<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n"));
    pf.print(F("<!DOCTYPE profile SYSTEM \"profile.dtd\">\n"));
    pf.print(F("<!--\n"
               "  ================================================================\n"));
    pf.print(isUnknown ? F("  SSSAL  AUTO-GENERATED PROFILE   (UNKNOWN ECU)\n")
                       : F("  SSSAL  AUTO-GENERATED DEFAULT PROFILE\n"));
    pf.print(F("  ================================================================\n\n"));
    pf.print(isUnknown
        ? F("  Created because the connected ECU ID was NOT found in the RomRaider\n"
            "  definitions. Only standard SSM \"P\" parameters are available here;\n"
            "  ECU-specific \"E\" parameters are not.\n\n")
        : F("  Created because no logger.xml was present. Lists every parameter the\n"
            "  ECU reported as supported, with a basic set enabled.\n\n"));
    snprintf(hdr, sizeof(hdr), "      ECU ID : %s\n      Created: %04u-%02u-%02u %02u:%02u:%02u\n\n",
             g_romId, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    pf.print(hdr);
    pf.print(F("  ================================================================\n"
               "  ENABLE / DISABLE A PARAMETER\n"
               "  ================================================================\n"
               "  To LOG a parameter, add  livedata=\"selected\"  to its line.\n"
               "  To stop, delete that attribute. Disabled parameters cost nothing;\n"
               "  they are never requested from the ECU.\n\n"
               "  ================================================================\n"
               "  SAMPLE RATE\n"
               "  ================================================================\n"
               "  Every ENABLED parameter is polled each sample over the 4800 baud\n"
               "  K-line, so the MORE you enable, the SLOWER the log rate. Enable\n"
               "  only what you need.\n\n"
               "  ================================================================\n"
               "  CURRENTLY DETECTED PARAMETERS    ( * = enabled by default )\n"
               "  ================================================================\n"));
    emitProfileSection(pf, 'i');
    pf.print(F("  ================================================================\n-->\n"));

    pf.print(F("<profile protocol=\"SSM\">\n    <parameters>\n"));
    emitProfileSection(pf, 'p');
    pf.print(F("    </parameters>\n    <switches>\n"));
    emitProfileSection(pf, 's');
    pf.print(F("    </switches>\n</profile>\n"));

    pf.sync();
    pf.close();
    return true;
}

// Read the embedded "ECU ID : <rom>" marker from unknownecu.xml; true if it targets
// the current ECU. Decides preserve (same ECU) vs overwrite (different unknown ECU).
static bool unknownEcuFileMatches() {
    SdFile f;
    if (!f.open(UNKNOWNECU_FILE, O_READ)) return false;
    char line[LINE_BUF_SIZE];
    bool match = false;
    uint8_t n = 0;
    while (n++ < 40 && f.fgets(line, sizeof(line)) > 0) {   // marker sits near the top
        char *p = strstr(line, "ECU ID");
        if (!p) continue;
        p = strchr(p, ':');
        if (!p) break;
        for (p++; *p == ' '; p++) {}
        char rom[16]; uint8_t i = 0;
        while (*p && *p != ' ' && *p != '\r' && *p != '\n' && i < sizeof(rom)-1) rom[i++] = *p++;
        rom[i] = '\0';
        match = (strcmp(rom, g_romId) == 0);
        break;
    }
    f.close();
    return match;
}

// ---- CALCULATED param dep resolution ----

// Scan a CALC expr string for param ID references (uppercase-letter-led tokens).
// e.g. "(P12*60)/P8" → ["P12","P8"].  Skips the 'x' variable (lowercase).
// Returns the count of unique IDs found.
static uint8_t extractParamIdsFromExpr(const char *expr,
                                        char ids[][MAX_ID_LEN], uint8_t maxIds) {
    uint8_t count = 0;
    const char *p = expr;
    while (*p && count < maxIds) {
        // '[Pxx:units]' — the param ID is the token right after '['; the ':units'
        // part must NOT be scanned (e.g. kPa/hPa would yield a bogus "Pa" token).
        bool bracket = false;
        if (*p == '[') { bracket = true; p++; }

        if (*p >= 'A' && *p <= 'Z') {
            char id[MAX_ID_LEN]; uint8_t n = 0;
            while (n < MAX_ID_LEN - 1 &&
                   ((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
                    (*p >= '0' && *p <= '9') || *p == '_'))
                id[n++] = *p++;
            id[n] = '\0';
            if (bracket) { while (*p && *p != ']') p++; if (*p == ']') p++; }
            // Deduplicate; skip single-letter tokens (A/B/C/D are OBD byte vars)
            bool dup = false;
            for (uint8_t j = 0; j < count; j++)
                if (strcmp(ids[j], id) == 0) { dup = true; break; }
            if (!dup && n > 1) {
                strncpy(ids[count], id, MAX_ID_LEN - 1);
                ids[count][MAX_ID_LEN - 1] = '\0';
                count++;
            }
        } else if (!bracket) {
            p++;   // ordinary non-identifier char (if bracket, the '[' was already consumed)
        }
    }
    return count;
}

// Load a dep param from address_map.csv into g_sel[] as a hidden entry.
// If depId is already in g_sel[] (selected or previously loaded), returns true.
// If not found in cache, returns false (dep will evaluate to 0.0).
static bool loadDepFromCache(const char *depId) {
    // Already present?
    for (uint8_t i = 0; i < g_numSel; i++)
        if (strcmp(g_sel[i].id, depId) == 0) return true;

    if (g_numSel >= MAX_SELECTED) {
        Serial.print(F("[CALC] dep slot full, skipping: ")); Serial.println(depId);
        return false;
    }

    SdFile f;
    if (!f.open(ADDR_MAP_FILE, O_READ)) return false;

    char line[LINE_BUF_SIZE];
    // Skip 4 header lines (DEBUG=, ECU=, DEFS=, column header)
    for (uint8_t h = 0; h < 4; h++) {
        if (f.fgets(line, LINE_BUF_SIZE) <= 0) { f.close(); return false; }
    }

    bool found = false;
    while (f.fgets(line, LINE_BUF_SIZE) > 0) {
        uint8_t idLen = strlen(depId);
        if (strncmp(line, depId, idLen) != 0 || line[idLen] != ',') continue;

        uint8_t si = g_numSel;
        strncpy(g_sel[si].id, depId, MAX_ID_LEN - 1);
        g_sel[si].id[MAX_ID_LEN - 1] = '\0';

        char f1[MAX_NAME_LEN], f2[12], f3[4], f4[16], f5[8], f6[MAX_EXPR_LEN];
        csvField(line, 1, f1, sizeof(f1));
        csvField(line, 2, f2, sizeof(f2));
        csvField(line, 3, f3, sizeof(f3));
        csvField(line, 4, f4, sizeof(f4));
        csvField(line, 5, f5, sizeof(f5));
        csvField(line, 6, f6, sizeof(f6));

        strncpy(g_sel[si].name, f1, MAX_NAME_LEN - 1);
        g_sel[si].name[MAX_NAME_LEN - 1] = '\0';

        uint8_t bn = 0;
        g_sel[si].ptype  = strToType(f4, &bn);
        g_sel[si].bitNum = bn;
        g_sel[si].endian = (strcmp(f5, "little") == 0) ? END_LITTLE : END_BIG;
        strncpy(g_sel[si].expr, f6, MAX_EXPR_LEN - 1);
        g_sel[si].expr[MAX_EXPR_LEN - 1] = '\0';
        // Restore escaped commas (if(a,b,c) args were stored as '|')
        for (char *e = g_sel[si].expr; *e; e++) if (*e == '|') *e = ',';

        if (strcmp(f2, "NOTFOUND") == 0 || strcmp(f2, "CALCULATED") == 0) {
            g_sel[si].address = 0;
            g_sel[si].len     = 0;
        } else {
            g_sel[si].address = (uint32_t)strtoul(f2, NULL, 16);
            g_sel[si].len     = (uint8_t)atoi(f3);
        }

        g_sel[si].fetchIdx      = FETCH_NONE;
        g_sel[si].lastVal       = 0.0f;
        g_sel[si].divZeroLogged = false;
        g_sel[si].hidden        = true;   // dep-only: fetched but not in CSV
        g_numSel++;
        found = true;

        Serial.print(F("[CALC] loaded dep: ")); Serial.println(depId);
        break;
    }
    f.close();

    if (!found) {
        Serial.print(F("[CALC] dep not in cache (evaluates to 0): "));
        Serial.println(depId);
    }
    return found;
}

// ---- Build fetch list ----

// Populate g_fetch[] from g_sel[], including hidden dep params for T_CALC entries.
// Order: (1) resolve dep params into g_sel[] as hidden entries;
//        (2) assign fetch slots for all non-CALC/non-NONE params;
//        (3) call ssmBuildBatchRequest(0x01) for fast poll.
void buildFetchList() {
    // Step 1: load dep params for every T_CALC entry.
    // Loop over a snapshot of g_numSel — loadDepFromCache may extend it.
    uint8_t selSnapshot = g_numSel;
    for (uint8_t i = 0; i < selSnapshot; i++) {
        if (g_sel[i].ptype != T_CALC) continue;
        char depIds[MAX_DEPS][MAX_ID_LEN];
        uint8_t nDeps = extractParamIdsFromExpr(g_sel[i].expr, depIds, MAX_DEPS);
        for (uint8_t d = 0; d < nDeps; d++)
            loadDepFromCache(depIds[d]);
    }

    // Step 2: assign fetch slots for all params (including newly-loaded hidden deps).
    // totalBytes tracks the SSM response data byte count = number of addresses in the
    // request (multi-byte params expand to consecutive addresses).
    g_numFetch = 0;
    uint16_t totalBytes = 0;
    for (uint8_t i = 0; i < g_numSel; i++) {
        if (g_sel[i].ptype == T_NONE || g_sel[i].ptype == T_CALC) {
            g_sel[i].fetchIdx = FETCH_NONE;
            continue;
        }
        // Deduplicate: reuse an existing slot if same address+len
        uint8_t existing = FETCH_NONE;
        for (uint8_t j = 0; j < g_numFetch; j++) {
            if (g_fetch[j].address == g_sel[i].address &&
                g_fetch[j].len     == g_sel[i].len) {
                existing = j; break;
            }
        }
        if (existing != FETCH_NONE) {
            g_sel[i].fetchIdx = existing;
        } else {
            // Overflow guards: max slots, and (SSM only) the per-request address limit.
            // A param that can't fit gets fetchIdx=FETCH_NONE → logs 0.00.
            if (g_numFetch >= MAX_FETCH ||
                (g_protocol == PROTO_SSM && totalBytes + g_sel[i].len > MAX_SSM_ADDR)) {
                g_sel[i].fetchIdx = FETCH_NONE;
                continue;
            }
            g_fetch[g_numFetch].address = g_sel[i].address;
            g_fetch[g_numFetch].len     = g_sel[i].len;
            g_fetch[g_numFetch].dataOff = 0;
            g_sel[i].fetchIdx = g_numFetch;
            g_numFetch++;
            totalBytes += g_sel[i].len;
        }
    }

    // SSM: build the batch request packet now (OBD polls per-PID in the streaming loop)
    if (g_protocol == PROTO_SSM) ssmBuildBatchRequest(0x01);
}

// ============================================================
// === SD OPS ===
// ============================================================

// ---- status.log ----

// Append one line to status.log: "YYYY-MM-DD HH:MM:SS,EVENT,detail\n"
// Written immediately (no buffering) — these are rare critical events.
void writeStatusLog(const char *event, const char *detail) {
    SdFile f;
    if (!f.open(STATUS_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND)) return;

    DateTime now = queryRTC();    // zeros if RTC failed
    char line[160];
    snprintf(line, sizeof(line),
             "%04d-%02d-%02d %02d:%02d:%02d,%s,%s\n",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second(),
             event, detail ? detail : "");
    f.print(line);
    f.sync();
    f.close();

    Serial.print(F("[STATUS] ")); Serial.print(event);
    if (detail && detail[0]) { Serial.print(' '); Serial.print(detail); }
    Serial.println();
}

// ---- verbose debug log ----

// Read the DEBUG= flag from the first line of address_map.csv into g_verboseDebug.
// Safe to call any time (even before the cache exists). Leaves g_verboseDebug
// unchanged-to-false if the file is missing or the first line is not DEBUG=.
// Called early in setup() so boot-sequence steps can be logged on all but the
// very first boot (when the cache does not yet exist).
void readDebugFlag() {
    g_verboseDebug = false;
    SdFile f;
    if (!f.open(ADDR_MAP_FILE, O_READ)) return;
    char line[40];
    if (f.fgets(line, sizeof(line)) > 0) {
        line[strcspn(line, "\r\n")] = '\0';
        if (strncmp(line, "DEBUG=", 6) == 0)
            g_verboseDebug = (strcmp(line + 6, "true") == 0 || line[6] == '1');
    }
    f.close();
}

// Append a timestamped line to verboselog.txt when g_verboseDebug is true.
// Format: "<millis>\t<msg>\n". millis() only (no RTC/I2C) — kept light for
// high-frequency per-step logging. No-op when verbose debug is off.
// NOTE: opens/syncs/closes per call — diagnostic use only; slows the loop.
// Future work: add vlog() calls at every step of the program.
void vlog(const char *msg) {
    if (!g_verboseDebug) return;
    SdFile f;
    if (!f.open(VERBOSE_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND)) return;
    char line[160];
    snprintf(line, sizeof(line), "%lu\t%s\n", (unsigned long)millis(), msg);
    f.print(line);
    f.sync();
    f.close();
}

// Two-part variant: "<millis>\t<msg> <detail>\n". For logging a value with a label.
void vlog2(const char *msg, const char *detail) {
    if (!g_verboseDebug) return;
    SdFile f;
    if (!f.open(VERBOSE_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND)) return;
    char line[200];
    snprintf(line, sizeof(line), "%lu\t%s %s\n",
             (unsigned long)millis(), msg, detail ? detail : "");
    f.print(line);
    f.sync();
    f.close();
}

// Append a blank separator line between sessions.
static void statusLogBlankLine() {
    SdFile f;
    if (!f.open(STATUS_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND)) return;
    f.print("\n");
    f.sync();
    f.close();
}

// Check status.log size; delete if > STATUS_LOG_MAX_BYTES (2KB).
// Must be called BEFORE any entries are written for this session.
void rotateStatusLog() {
    if (!g_sd.exists(STATUS_LOG_FILE)) return;
    SdFile f;
    if (!f.open(STATUS_LOG_FILE, O_READ)) return;
    uint32_t sz = f.fileSize();
    f.close();
    if (sz > STATUS_LOG_MAX_BYTES) {
        g_sd.remove(STATUS_LOG_FILE);
        Serial.println(F("[SD] status.log rotated"));
    }
}

// ---- Regenerate cache (implementation) ----

// Full cache regeneration: scan definitions file, write new address_map.csv.
bool regenCache() {
    // Open / create address_map.csv for writing
    if (g_sd.exists(ADDR_MAP_FILE)) g_sd.remove(ADDR_MAP_FILE);
    SdFile cf;
    if (!cf.open(ADDR_MAP_FILE, O_WRONLY | O_CREAT | O_TRUNC)) {
        writeStatusLog("CACHE_WRITE_FAIL", "address_map.csv write failed");
        return false;
    }

    // Write header lines. DEBUG= is ALWAYS written false on creation/regeneration —
    // the user edits it to true manually when they want verbose tracing. (A user's
    // DEBUG=true does NOT survive a cache regen; they re-enable it after regen.)
    char hdr[64];
    cf.print(F("DEBUG=false\n"));
    snprintf(hdr, sizeof(hdr), "ECU=%s\n", g_romId);
    cf.print(hdr);
    snprintf(hdr, sizeof(hdr), "DEFS=%s\n", g_defsFilename);
    cf.print(hdr);
    cf.print(F("param_id,name,address,len,type,endian,expr,units,deps\n"));

    // Build the COMPLETE per-ECU dictionary (every supported param, all unit variants).
    // Params the ECU doesn't support are simply omitted; a later selection of an
    // unsupported param is handled as NOTFOUND at load time (no regen).
    parseDefinitions(cf);

    cf.sync();
    cf.close();
    return true;
}

// ---- Log file ops ----

// Write CSV header row: "millis,Name1 (ID1),Name2 (ID2),..."
static void writeLogHeader() {
    writeToBuffer("millis");
    for (uint8_t i = 0; i < g_numSel; i++) {
        if (g_sel[i].hidden) continue;   // dep-only params not in CSV output
        writeToBuffer(",");
        writeToBuffer(g_sel[i].name);
        writeToBuffer(" (");
        writeToBuffer(g_sel[i].id);
        writeToBuffer(")");
    }
    writeToBuffer("\n");
}

// Append str to g_writeBuf; flush (write one sector) when full.
void writeToBuffer(const char *str) {
    while (*str) {
        g_writeBuf[g_writeBufPos++] = *str++;
        if (g_writeBufPos >= WRITE_BUF_SIZE) {
            // write() returns size_t (bytes written); short count = error (e.g. SD full)
            if (g_logFile.write(g_writeBuf, WRITE_BUF_SIZE) != WRITE_BUF_SIZE)
                g_sdWriteError = true;
            g_writeBufPos = 0;
        }
    }
}

// Flush any remaining bytes in g_writeBuf to the log file.
void flushBuffer() {
    if (g_writeBufPos > 0) {
        // write() returns size_t (bytes written); short count = error (e.g. SD full)
        if (g_logFile.write(g_writeBuf, g_writeBufPos) != g_writeBufPos)
            g_sdWriteError = true;
        g_writeBufPos = 0;
    }
}

// Open a new log CSV file; returns true on success.
// filename must be a 24-char NNNN_YYYYMMDD_HHMMSS.csv string.
static bool openLogFile(const char *filename) {
    char path[40];
    snprintf(path, sizeof(path), "/logger/%s", filename);
    if (!g_logFile.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return false;
    if (!g_logFile.preAllocate(PREALLOCATE_BYTES)) {
        // Non-fatal: logging works without pre-allocation, just with higher SD latency
        writeStatusLog("PREALLOC_WARN", "50MB pre-allocation failed — SD may be nearly full");
    }
    g_writeBufPos  = 0;
    g_sdWriteError = false;
    g_lastSync     = millis();
    writeLogHeader();
    return true;
}

// Flush, truncate to actual size, sync, and close the log file.
static void closeLogFile() {
    flushBuffer();
    g_logFile.truncate();
    g_logFile.sync();
    g_logFile.close();
}

// Periodic sync — call inside the streaming loop.
static void maybeSyncFile() {
    if (millis() - g_lastSync >= SYNC_INTERVAL_MS) {
        flushBuffer();
        g_logFile.sync();
        g_lastSync = millis();
    }
}

// Pass 2 CALC evaluation + CSV row write.  Called after Pass 1 (SSM or OBD decode)
// has filled vals[] and stored lastVal for all directly-fetched params.
static void writeSampledRow(float *vals) {
    // Pass 2: evaluate T_CALC params (deps already have lastVal from Pass 1).
    // NOTE: if a CALC param depends on another CALC param whose g_sel[] index is
    // higher (i.e. evaluated later in this loop), it will read stale lastVal from
    // the previous sample.  This cannot happen with current RomRaider definitions
    // (all CALC deps are raw-fetch P params) — flag here if that changes.
    for (uint8_t i = 0; i < g_numSel; i++) {
        if (g_sel[i].ptype != T_CALC) continue;
        bool divZ = false;
        vals[i] = evalCalcExpr(g_sel[i].expr, &divZ);
        if (divZ && !g_sel[i].divZeroLogged) {
            g_sel[i].divZeroLogged = true;
            char det[80];  // id(23)+" divisor=0 at T="(16)+10+2+3+13+NUL = 68 min
            snprintf(det, sizeof(det), "%s divisor=0 at T=%lums — logging zero",
                     g_sel[i].id, (unsigned long)millis());
            writeStatusLog("DIV_ZERO", det);
        }
        g_sel[i].lastVal = vals[i];
    }

    // Format and buffer CSV row
    char cell[20];
    snprintf(cell, sizeof(cell), "%lu", (unsigned long)millis());
    writeToBuffer(cell);
    for (uint8_t i = 0; i < g_numSel; i++) {
        if (g_sel[i].hidden) continue;
        writeToBuffer(",");
        if (isRawHexType(g_sel[i].ptype)) {
            // Raw integer → exact, zero-padded hex (width = byte count × 2), so
            // multi-byte immo registers keep every bit and unpack byte-by-byte.
            snprintf(cell, sizeof(cell), "0x%0*lX",
                     g_sel[i].len * 2, (unsigned long)g_rawVals[i]);
        } else {
            snprintf(cell, sizeof(cell), "%.2f", vals[i]);
        }
        writeToBuffer(cell);
    }
    writeToBuffer("\n");
    maybeSyncFile();
}

// ============================================================
// === RTC OPS ===
// ============================================================

// Query the DS1307 RTC. Returns current DateTime; returns a
// zeros DateTime (year=2000) if RTC is not available.
DateTime queryRTC() {
    if (!g_rtcOk) return DateTime(2000, 1, 1, 0, 0, 0);
    return g_rtc.now();
}


// Read g_sessionNum from counter.txt.
// Falls back to scanning logger/ for highest NNNN_ prefix (+1) or 1.
static void readCounter() {
    SdFile f;
    if (f.open(COUNTER_FILE, O_READ)) {
        char buf[8] = {0};
        f.fgets(buf, sizeof(buf));
        f.close();
        int n = atoi(buf);
        g_sessionNum = (n > 0 && n <= COUNTER_MAX) ? (uint16_t)n : 1;
        return;
    }
    // counter.txt missing — scan for highest NNNN_ prefix
    SdFile dir, entry;
    uint16_t highest = 0;
    if (dir.open("/logger")) {
        char fname[32];
        while (entry.openNext(&dir, O_READ)) {
            entry.getName(fname, sizeof(fname));
            entry.close();
            if (fname[0] >= '0' && fname[0] <= '9') {
                uint16_t n = (uint16_t)atoi(fname);
                if (n > highest) highest = n;
            }
        }
        dir.close();
    }
    g_sessionNum = (highest < COUNTER_MAX) ? highest + 1 : 1;
}

// Increment g_sessionNum and write back to counter.txt.
static void incrementCounter() {
    g_sessionNum++;
    if (g_sessionNum > COUNTER_MAX) g_sessionNum = 1;
    SdFile f;
    if (f.open(COUNTER_FILE, O_WRONLY | O_CREAT | O_TRUNC)) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%04u\n", g_sessionNum);
        f.print(buf);
        f.sync();
        f.close();
    }
}

// Build log filename into out[]: "NNNN_YYYYMMDD_HHMMSS.csv"
// Uses g_sessionNum and current RTC time.
static void buildFilename(char *out, uint8_t maxLen) {
    DateTime now = queryRTC();
    snprintf(out, maxLen, "%04u_%04u%02u%02u_%02u%02u%02u.csv",
             g_sessionNum,
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
}

// ============================================================
// === MAIN ===
// ============================================================

// ---- AGT1 low-power sleep (RA4M1) ----
// Used in PHASE_IDLE: ~40µA during sleep vs ~7.5mA with WFI.
// Sources: TriodeGirl (WFI), Armin/jwshort (Software Standby + AGT1).

#define AGT1_BASE    0x40084100UL
#define AGT1_AGT     ((volatile uint16_t*)(AGT1_BASE + 0x00))
#define AGT1_AGTCR   ((volatile uint8_t*) (AGT1_BASE + 0x08))
#define AGT1_AGTIOC  ((volatile uint8_t*) (AGT1_BASE + 0x09))
#define AGT1_AGTISR  ((volatile uint8_t*) (AGT1_BASE + 0x0A))
#define AGT1_AGTCMSR ((volatile uint8_t*) (AGT1_BASE + 0x0B))
#define AGT1_AGTIOSEL ((volatile uint8_t*)(AGT1_BASE + 0x0F))
#define AGT1_AGTMR1  ((volatile uint8_t*) (AGT1_BASE + 0x18))
#define AGT1_AGTMR2  ((volatile uint8_t*) (AGT1_BASE + 0x19))
#define MSTP_MSTPCRD ((volatile uint32_t*)0x4001E01CUL)

// Configure AGT1 to fire after `counts` AGTLCLK/128 ticks.
// AGTLCLK = 32.768 kHz / 128 = 256 Hz → 2 min = 30720 counts.
// Module clock MUST be enabled first — accessing registers with MSTPD2 set is undefined.
static void setupAGT1Wakeup(uint16_t counts) {
    *MSTP_MSTPCRD &= ~(1UL << 2);   // enable AGT1 module clock (MSTPD2) — MUST be first
    *AGT1_AGTCR    = 0;             // stop timer, clear flags
    *AGT1_AGTMR1   = 0;
    *AGT1_AGTMR2   = 0;
    *AGT1_AGTIOC   = 0;
    *AGT1_AGTISR   = 0;
    *AGT1_AGTCMSR  = 0;
    *AGT1_AGTIOSEL = 0;
    *AGT1_AGT      = counts;
    *AGT1_AGTMR1   = (4 << 4);      // AGTLCLK source (value 4 = AGTSRC_AGTLCLK)
    *AGT1_AGTMR2   = 7;             // /128 prescaler
    *AGT1_AGTCR    = 1;             // TSTART — begin countdown
}

// Clear AGT1 underflow flag after wakeup.
// TUNDF (bit 4 of AGTCR) is write-1-to-clear — must write 1, not 0.
static void clearAGT1Flags() {
    *AGT1_AGTCR = (uint8_t)0x10;   // W1C: write 1 to bit 4 (TUNDF) to clear it
}

// Enter Software Standby; wake on AGT1 underflow.
// NOTE: WUPEN bit 24 for AGT1 underflow — VERIFY against RA4M1 manual on hw arrival.
static void softwareStandbySleep() {
    R_ICU->WUPEN  |= (1UL << 24);   // enable AGT1 underflow wakeup
    R_SYSTEM->PRCR = 0xA503;        // unlock standby registers (CRITICAL)
    R_SYSTEM->SBYCR_b.SSBY = 1;     // select Software Standby mode
    R_DTC->DTCST_b.DTCST = 0;       // disable DTC
    R_SYSTEM->OSTDCR_b.OSTDE = 0;   // disable oscillation stop detection
    asm volatile("wfi");             // sleep until interrupt
    R_SYSTEM->PRCR = 0xA500;        // re-lock standby registers
}

// ---- Fatal error halt ----

static void fatalHalt(const char *event, const char *detail) {
    writeStatusLog(event, detail);
    Serial.print(F("[FATAL] ")); Serial.print(event);
    if (detail && detail[0]) { Serial.print(' '); Serial.println(detail); }
    else Serial.println();
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH); delay(250);
        digitalWrite(LED_BUILTIN, LOW);  delay(250);
    }
}

// ---- Per-engine-start sequence ----
// Called when ECU init response received in PHASE_SEARCH or PHASE_IDLE.
// Opens a new log file and transitions to PHASE_STREAMING.

static bool g_ecuIdNotFound = false;  // set in handleEcuFound when ROM matched no ecuparam (unknown ECU)

static void handleEcuFound() {
    char detail[48];
    snprintf(detail, sizeof(detail), "ROM ID=%s", g_romId);
    writeStatusLog("ECU_FOUND", detail);
    Serial.print(F("[ECU] ")); Serial.println(g_romId);
    vlog2("[ECU] found ROM ID", g_romId);

    // Check cache headers (also refreshes g_verboseDebug from the DEBUG= line)
    uint8_t hdr = checkCacheHeaders();
    bool needRegen = (hdr != 3);
    if (needRegen) {
        const char *reason;
        char reasonBuf[64];
        if      (hdr == 0) reason = "reason=FILE_MISSING";
        else if (hdr == 1) { snprintf(reasonBuf, sizeof(reasonBuf),
                             "reason=ROM_ID_CHANGE new=%s", g_romId);
                             reason = reasonBuf; }
        else               reason = "reason=DEFS_CHANGE";
        writeStatusLog("CACHE_REGEN", reason);
        g_ecuIdNotFound = false;
        if (!regenCache()) { fatalHalt("CACHE_WRITE_FAIL", "address_map.csv write failed"); return; }
    } else {
        writeStatusLog("CACHE_HIT", "address_map.csv valid");
    }

    // ---- Decide which profile to log from (selection-INDEPENDENT) ----
    // "Known" ECU = its ROM matched at least one <ecuparam> → the cache holds E rows.
    // This no longer depends on what the user selected, so it is reliable even when
    // no E params are enabled.
    bool ecuKnown = cacheHasEParams();
    g_ecuIdNotFound = !ecuKnown;

    if (!ecuKnown) {
        // UNKNOWN ECU → log from unknownecu.xml, ignoring any logger.xml. Preserve the
        // file when it already targets this same ROM (keeps hand-enabled params across
        // reboots); regenerate it when a different unknown ECU appears.
        char det[64];
        snprintf(det, sizeof(det), "ROM ID=%s not in definitions — P params only", g_romId);
        writeStatusLog("ECU_ID_NOT_FOUND", det);
        Serial.println(F("[ECU] ECU ID not in definitions — using unknownecu.xml"));

        if (g_sd.exists(UNKNOWNECU_FILE) && unknownEcuFileMatches()) {
            writeStatusLog("PROFILE_KEEP", "unknownecu.xml preserved (same ECU)");
        } else {
            writeStatusLog("PROFILE_GEN", generateProfile(UNKNOWNECU_FILE, true)
                                          ? "unknownecu.xml created" : "unknownecu.xml FAILED");
        }
        parseProfile(UNKNOWNECU_FILE);
    } else if (!g_haveLoggerXml) {
        // KNOWN ECU but no logger.xml → generate a default one (never overwrites an
        // existing logger.xml — we only get here when none was present at boot).
        writeStatusLog("PROFILE_GEN", generateProfile(PROFILE_FILE, false)
                                      ? "default logger.xml created" : "logger.xml FAILED");
        g_haveLoggerXml = true;
        parseProfile(PROFILE_FILE);
    } else {
        // KNOWN ECU + logger.xml present → keep the selection parsed at boot.
        if (g_numSel == 0) { fatalHalt("NO_PARAMS", "logger.xml has no selected parameters"); return; }
    }

    // Load the (now finalized) selection from the complete-dictionary cache. This
    // never needs a regen for a "new" selection — the dictionary already holds every
    // supported param; an unsupported (id,units) is self-marked NOTFOUND in the loader.
    // A false return means only a structurally bad cache → regen once and reload.
    if (!loadCacheForSelected()) {
        writeStatusLog("CACHE_REGEN", "reason=CACHE_UNREADABLE");
        if (!regenCache()) { fatalHalt("CACHE_WRITE_FAIL", ""); return; }
        loadCacheForSelected();   // second attempt; accept whatever state we get
    }

    // Build filename and open log file BEFORE incrementing counter.
    // Counter is only incremented if the file opens successfully — prevents
    // skipped session numbers on SD-full or other open failures.
    buildFilename(g_logFilename, sizeof(g_logFilename));

    if (!openLogFile(g_logFilename)) {
        writeStatusLog("LOG_WRITE_FAIL", g_logFilename);
        // Non-fatal — stay in PHASE_SEARCH and retry on next engine start
        return;
    }

    incrementCounter();   // committed only after successful file open

    Serial.print(F("[LOG] ")); Serial.println(g_logFilename);

    buildFetchList();
    if (g_protocol == PROTO_SSM) ssmSendBatchRequest();   // OBD polls per-PID in streaming loop

    g_firstPacket = true;
    g_phase = PHASE_STREAMING;
}

// ============================================================
// setup()
// ============================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(SERIAL_DEBUG_BAUD);
    Serial1.begin(SSM_BAUD, SERIAL_8N1);

    // ---- SD init ----
    if (!g_sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
        Serial.println(F("[FATAL] SD_INIT_FAIL"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW);  delay(200);
        }
    }
    g_sd.mkdir("/logger");
    Serial.println(F("[BOOT] SD OK"));

    // ---- Verbose debug flag (read early so boot steps can be logged) ----
    // On first-ever boot address_map.csv does not exist yet, so g_verboseDebug
    // stays false until the cache is generated and the user edits DEBUG=true.
    readDebugFlag();
    vlog("[BOOT] SD init OK, verbose debug enabled");

    // ---- Rotate status.log if oversized (before writing any entries) ----
    rotateStatusLog();
    statusLogBlankLine();

    // ---- Session counter ----
    readCounter();

    // ---- RTC ----
    g_rtcOk = g_rtc.begin();
    vlog(g_rtcOk ? "[BOOT] RTC OK" : "[BOOT] RTC not responding");

    // ---- Find definitions file ----
    if (!findDefsFile()) {
        fatalHalt("DEFS_MISSING", "no logger_*.xml found in logger/");
        return;
    }
    vlog2("[BOOT] defs file:", g_defsFilename);

    // ---- Parse profile (logger.xml) if present ----
    // A missing logger.xml is no longer fatal: we default to SSM and auto-generate a
    // profile after the ECU is detected (default logger.xml if the ECU is known,
    // unknownecu.xml if not). An empty existing logger.xml is handled post-detection.
    g_haveLoggerXml = g_sd.exists(PROFILE_FILE);
    if (g_haveLoggerXml) {
        if (!parseProfile(PROFILE_FILE))
            writeStatusLog("PROFILE_EMPTY", "logger.xml has no selected params yet");
    } else {
        g_protocol = PROTO_SSM;
        g_numSel   = 0;
        writeStatusLog("PROFILE_MISSING", "no logger.xml — will auto-generate after ECU detect");
    }

    // ---- Protocol check and hardware init ----
    if (g_protocol == PROTO_OBD) {
        if (!obdCanInit())
            fatalHalt("CAN_INIT_FAIL", "CAN bus init failed — check TJA1050 on D4/D5");
        Serial.println(F("[BOOT] CAN OK (OBD mode)"));
    } else if (g_protocol != PROTO_SSM) {
        fatalHalt("PROTO_NOT_SUPPORTED", "only SSM and OBD protocols are implemented");
        return;
    }

    // ---- Log startup status ----
    char detail[48];
    snprintf(detail, sizeof(detail), "%u params selected", g_numSel);
    writeStatusLog("PROFILE_OK", detail);
    writeStatusLog("DEFS_OK", g_defsFilename);
    if (!g_rtcOk)
        writeStatusLog("RTC_FAIL", "DS1307 not responding — timestamps unavailable");

    Serial.print(F("[BOOT] profile OK: ")); Serial.print(g_numSel); Serial.println(F(" params"));
    Serial.print(F("[BOOT] defs: ")); Serial.println(g_defsFilename);

    digitalWrite(LED_BUILTIN, HIGH);   // booted and ready

    // Enter Phase 1 detection
    g_phase = PHASE_SEARCH;
}

// ============================================================
// loop()
// ============================================================

static uint32_t g_searchStartMs = 0;

// Probe the ECU using the current protocol.  Returns true if the ECU responded.
// For SSM: sends init packet + reads init response (fills g_romId, g_ecuInitRaw).
// For OBD: sends PID 0x00, waits for response, sets g_romId = "OBD_GENERIC".
static bool protocolProbe() {
    if (g_protocol == PROTO_OBD) return obdProbe();
    ssmSendInitPacket();
    uint8_t rawResp[96]; uint8_t respLen = 0;
    return ssmReadInitResponse(rawResp, sizeof(rawResp), &respLen);
}

// Write the terminal LOGGING_STARTED* entry to status.log on first sample.
static void writeFirstPacketStatus() {
    char det[72];
    uint8_t nUndef = 0, nVis = 0;
    char undefId[MAX_ID_LEN] = {0};
    for (uint8_t i = 0; i < g_numSel; i++) {
        if (g_sel[i].hidden) continue;
        nVis++;
        if (g_sel[i].ptype == T_NONE) {
            nUndef++;
            if (!undefId[0]) strncpy(undefId, g_sel[i].id, MAX_ID_LEN-1);
        }
    }
    if (g_ecuIdNotFound) {
        uint8_t nP = 0, nE = 0;
        for (uint8_t i = 0; i < g_numSel; i++) {
            if (g_sel[i].hidden) continue;
            if (g_sel[i].id[0] == 'P') nP++; else nE++;
        }
        snprintf(det, sizeof(det), "file=%s p_params=%u e_params_skipped=%u",
                 g_logFilename, nP, nE);
        writeStatusLog("LOGGING_STARTED_P_ONLY", det);
    } else if (nUndef > 0) {
        snprintf(det, sizeof(det), "file=%s undefined=%s", g_logFilename, undefId);
        writeStatusLog("LOGGING_STARTED_WITH_UNDEFINED", det);
    } else {
        snprintf(det, sizeof(det), "file=%s params=%u", g_logFilename, (unsigned)nVis);
        writeStatusLog("LOGGING_STARTED", det);
    }
}

void loop() {

    // ================================================================
    // PHASE_SEARCH — fast retry loop (up to PHASE1_SEARCH_DURATION_MS)
    // ================================================================
    if (g_phase == PHASE_SEARCH) {
        if (g_searchStartMs == 0) g_searchStartMs = millis();

        if (protocolProbe()) {
            g_searchStartMs = 0;
            handleEcuFound();
            return;
        }

        if (millis() - g_searchStartMs >= PHASE1_SEARCH_DURATION_MS) {
            g_searchStartMs = 0;
            g_phase = PHASE_IDLE;
            Serial.println(F("[PHASE] Search timeout → IDLE (2-min sleep)"));
        }
    }

    // ================================================================
    // PHASE_IDLE — low-power sleep, probe on wakeup
    // ================================================================
    else if (g_phase == PHASE_IDLE) {
        uint32_t counts = (uint32_t)PHASE1_IDLE_SLEEP_MS * 256UL / 1000UL;
        if (counts > 65535) counts = 65535;
        setupAGT1Wakeup((uint16_t)counts);
        softwareStandbySleep();
        clearAGT1Flags();

        if (protocolProbe()) {
            // Pre-set PHASE_SEARCH before handleEcuFound(): if the file open fails
            // and it returns early, the next loop() iteration fast-retries rather
            // than sleeping another 2 minutes.
            g_phase = PHASE_SEARCH;
            handleEcuFound();
        }
    }

    // ================================================================
    // PHASE_STREAMING — log one sample per call
    // ================================================================
    else if (g_phase == PHASE_STREAMING) {
        float vals[MAX_SELECTED] = {0};
        bool timedOut = false;

        if (g_protocol == PROTO_OBD) {
            // OBD: send a CAN request and wait for a response per param individually.
            // The RomRaider OBD section assembles the PID's data bytes into 'x'
            // (big-endian, width per storagetype) and the expr operates on x — the
            // SAME model as SSM — so we reuse ssmDecodeSlot on the response bytes.
            // (A/B/C/D byte variables are NOT used by this definitions file.)
            for (uint8_t i = 0; i < g_numSel && !timedOut; i++) {
                if (g_sel[i].ptype == T_NONE || g_sel[i].ptype == T_CALC ||
                    g_sel[i].fetchIdx == FETCH_NONE) {
                    vals[i] = 0.0f;
                    g_sel[i].lastVal = 0.0f;
                    continue;
                }
                uint8_t pid = (uint8_t)(g_fetch[g_sel[i].fetchIdx].address & 0xFF);
                obdSendRequest(pid);
                uint8_t respBytes[4] = {0};   // A,B,C,D data bytes from the CAN frame
                if (!obdReadResponse(pid, respBytes, 2000)) {
                    timedOut = true;
                    break;
                }
                vals[i] = ssmDecodeSlot(respBytes, 0, g_sel[i].len, g_sel[i].ptype,
                                        g_sel[i].endian, g_sel[i].bitNum, g_sel[i].expr);
                g_sel[i].lastVal = vals[i];
                if (isRawHexType(g_sel[i].ptype))
                    g_rawVals[i] = assembleRawInt(respBytes, 0, g_sel[i].len, g_sel[i].endian);
            }
        } else {
            // SSM: read one batch response (ECU is streaming continuously)
            uint8_t data[MAX_BATCH_DATA];
            if (!ssmReadBatchResponse(data, PHASE2_WATCHDOG_MS)) {
                timedOut = true;
            } else {
                for (uint8_t i = 0; i < g_numSel; i++) {
                    if (g_sel[i].ptype == T_NONE) {
                        vals[i] = 0.0f; g_sel[i].lastVal = 0.0f;
                    } else if (g_sel[i].ptype == T_CALC || g_sel[i].fetchIdx == FETCH_NONE) {
                        vals[i] = 0.0f;
                    } else {
                        FetchSlot &fs = g_fetch[g_sel[i].fetchIdx];
                        vals[i] = ssmDecodeSlot(data, fs.dataOff,
                                                g_sel[i].len, g_sel[i].ptype,
                                                g_sel[i].endian, g_sel[i].bitNum,
                                                g_sel[i].expr);
                        g_sel[i].lastVal = vals[i];
                        if (isRawHexType(g_sel[i].ptype))
                            g_rawVals[i] = assembleRawInt(data, fs.dataOff,
                                                          g_sel[i].len, g_sel[i].endian);
                    }
                }
            }
        }

        if (timedOut) {
            writeStatusLog("STREAM_WATCHDOG", "no data — re-entering Phase 1");
            Serial.println(F("[PHASE] Watchdog → close log, SEARCH"));
            flushBuffer();
            closeLogFile();
            if (g_protocol == PROTO_SSM) ssmFlushRx();
            g_phase = PHASE_SEARCH;
            g_searchStartMs = 0;
            return;
        }

        // Terminal status log line on very first sample
        if (g_firstPacket) { g_firstPacket = false; writeFirstPacketStatus(); }

        // Pass 2 CALC + CSV write (shared between SSM and OBD)
        writeSampledRow(vals);

        // Check if any SD write failed during this sample
        if (g_sdWriteError) {
            g_sdWriteError = false;
            writeStatusLog("SD_WRITE_FAIL", g_logFilename);
            Serial.println(F("[PHASE] SD write error → close log, SEARCH"));
            flushBuffer();
            closeLogFile();
            if (g_protocol == PROTO_SSM) ssmFlushRx();
            g_phase = PHASE_SEARCH;
            g_searchStartMs = 0;
        }
    }
}
