// SsmInitTiming.cs — SSM pre-flight check + timing tool via J2534 (Tactrix OpenPort 2.0)
//
// PURPOSE: validate everything the Arduino K-Line logger depends on, BEFORE relying on
// the Arduino — so on-car troubleshooting is minimized. It:
//   • measures init (0xBF) + batch-read (0xA8) round-trip timing (firmware constants)
//   • decodes the ECU init response: ROM ID + capability bitmap
//   • maps the capability bits to supported P-parameter NAMES via logger_*.xml
//   • live-reads known params (RPM, coolant, battery, …) and decodes engineering values
//   • probes the ECU's max batch-read size (validates the firmware byte budget)
//
// Compile: C:\Windows\Microsoft.NET\Framework\v4.0.30319\csc.exe /platform:x86 /out:SsmInitTiming.exe SsmInitTiming.cs
// Run:     SsmInitTiming.exe   (run from the folder containing logger_*.xml for name mapping)
// Prereq:  Tactrix plugged into OBD2, key-on or engine running, RomRaider NOT open

using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

class SsmInitTiming
{
    // ── kernel32 ─────────────────────────────────────────────────────────────
    [DllImport("kernel32.dll", SetLastError=true)]
    static extern IntPtr LoadLibraryEx(string path, IntPtr file, uint flags);
    [DllImport("kernel32.dll")]
    static extern IntPtr GetProcAddress(IntPtr hMod, string name);

    // ── J2534 delegates ──────────────────────────────────────────────────────
    delegate int dOpen         (IntPtr pName, out uint devId);
    delegate int dClose        (uint devId);
    delegate int dReadVersion  (uint devId, StringBuilder fw, StringBuilder dll, StringBuilder api);
    delegate int dConnect      (uint devId, uint proto, uint flags, uint baud, out uint chanId);
    delegate int dDisconnect   (uint chanId);
    delegate int dIoctl        (uint id, uint ioctlId, IntPtr pIn, IntPtr pOut);
    delegate int dStartFilter  (uint chanId, uint fType, IntPtr pMask, IntPtr pPat, IntPtr pFlow, out uint filtId);
    delegate int dWriteMsgs    (uint chanId, IntPtr pMsg, ref uint n, uint tMs);
    delegate int dReadMsgs     (uint chanId, IntPtr pMsg, ref uint n, uint tMs);
    delegate int dGetLastError (StringBuilder msg);

    // ── PASSTHRU_MSG layout ──────────────────────────────────────────────────
    const int  MSG_SIZE  = 4152;
    const int  OFF_PROTO = 0;
    const int  OFF_RXST  = 4;
    const int  OFF_DSIZE = 16;
    const int  OFF_DATA  = 24;
    const uint TX_IND    = 0x10;   // RxStatus: this message is TX echo

    // ── J2534 constants ──────────────────────────────────────────────────────
    const uint ISO9141             = 3;   // J2534 protocol ID (was 5 — that's CAN!)
    const uint ISO9141_NO_CHECKSUM = 0x0200;   // J2534 PassThruConnect flag (was 0x04 — wrong)
    const uint SET_CONFIG          = 0x02;
    // J2534 Config parameter IDs (from RomRaider J2534Impl.Config — verified)
    const uint CFG_LOOPBACK        = 0x03;
    const uint CFG_P1_MAX          = 0x07;
    const uint CFG_P3_MIN          = 0x0A;
    const uint CFG_P4_MIN          = 0x0C;
    const uint CFG_PARITY          = 0x16;   // was 0x17 — that's BIT_SAMPLE_POINT, NOT parity
    const uint CFG_DATA_BITS       = 0x20;
    const uint NO_PARITY           = 0;
    const uint PASS_FILTER         = 1;

    // Use the registry-registered J2534 driver (the one RomRaider loads) — not the
    // EcuFlash-bundled copy, which is a different build that rejects SET_CONFIG params.
    const string DLL = @"C:\WINDOWS\SysWOW64\op20pt32.dll";

    // ── Batch read test addresses (low RAM, exist on all EJ ECUs) ────────────
    static readonly byte[][] TEST_ADDRS = {
        new byte[]{ 0x00, 0x00, 0x00 },
        new byte[]{ 0x00, 0x00, 0x01 },
        new byte[]{ 0x00, 0x00, 0x02 },
    };

    const int INIT_ITERS = 5;    // timing is well-characterised now; a few samples suffice
    const int READ_ITERS = 5;

    // ── J2534 function pointers ──────────────────────────────────────────────
    static dWriteMsgs   WriteMsgs;
    static dReadMsgs    ReadMsgs;
    static dGetLastError GetLastError;

    // ────────────────────────────────────────────────────────────────────────
    static void Main(string[] args)
    {
        bool brakeMode = (args.Length > 0 && args[0].ToLowerInvariant() == "brake");
        Log("=== SsmInitTiming — K-Line logger firmware timing reference ===");
        Log("DLL: " + DLL);
        Log("");

        IntPtr hMod = LoadLibraryEx(DLL, IntPtr.Zero, 0);
        if (hMod == IntPtr.Zero) { Log("LoadLibrary failed: " + Marshal.GetLastWin32Error()); return; }

        var Open        = Fn<dOpen>       (hMod, "PassThruOpen");
        var Close       = Fn<dClose>      (hMod, "PassThruClose");
        var ReadVersion = Fn<dReadVersion>(hMod, "PassThruReadVersion");
        var Connect     = Fn<dConnect>    (hMod, "PassThruConnect");
        var Disconnect  = Fn<dDisconnect> (hMod, "PassThruDisconnect");
        var Ioctl       = Fn<dIoctl>      (hMod, "PassThruIoctl");
        var StartFilter = Fn<dStartFilter>(hMod, "PassThruStartMsgFilter");
        WriteMsgs    = Fn<dWriteMsgs>  (hMod, "PassThruWriteMsgs");
        ReadMsgs     = Fn<dReadMsgs>   (hMod, "PassThruReadMsgs");
        GetLastError = Fn<dGetLastError>(hMod, "PassThruGetLastError");

        // ── Open device ──────────────────────────────────────────────────────
        uint devId;
        int r = Open(IntPtr.Zero, out devId);
        if (!Check(r, "PassThruOpen")) return;

        var fw = new StringBuilder(256); var dll2 = new StringBuilder(256); var api = new StringBuilder(256);
        ReadVersion(devId, fw, dll2, api);
        Log("Tactrix FW=" + fw + "  DLL=" + dll2);
        Log("");

        // ── Connect ISO9141, 4800 baud ───────────────────────────────────────
        uint chanId;
        r = Connect(devId, ISO9141, ISO9141_NO_CHECKSUM, 4800, out chanId);
        if (!Check(r, "PassThruConnect ISO9141/4800")) { Close(devId); return; }

        // ── Channel config — same params RomRaider uses for ISO9141/SSM, but set
        // ONE AT A TIME so an unsupported param on this OP2.0 firmware doesn't
        // abort the whole batch (and so we see exactly which one is rejected).
        // Framing params (PARITY, DATA_BITS) first as they matter most.
        uint[]   cfgP = { CFG_PARITY, CFG_DATA_BITS, CFG_P1_MAX, CFG_P3_MIN, CFG_P4_MIN, CFG_LOOPBACK };
        uint[]   cfgV = {  NO_PARITY,             0,          1,          1,          0,            1 };
        string[] cfgN = { "PARITY=0", "DATA_BITS=8", "P1_MAX=1", "P3_MIN=1", "P4_MIN=0", "LOOPBACK=1" };
        for (int k = 0; k < cfgP.Length; k++)
        {
            byte[] c1 = new byte[16];
            PutU32(c1, 0, 1);            // NumOfParams = 1
            PutU32(c1, 8, cfgP[k]);      // SCONFIG.Parameter
            PutU32(c1, 12, cfgV[k]);     // SCONFIG.Value
            GCHandle h1 = GCHandle.Alloc(c1, GCHandleType.Pinned);
            PutU32(c1, 4, (uint)(h1.AddrOfPinnedObject().ToInt64() + 8));  // ConfigPtr → SCONFIG
            int rc = Ioctl(chanId, SET_CONFIG, h1.AddrOfPinnedObject(), IntPtr.Zero);
            h1.Free();
            Check(rc, "SET_CONFIG " + cfgN[k]);
        }

        // ── Pass-all filter ──────────────────────────────────────────────────
        byte[] maskMsg = new byte[MSG_SIZE];
        byte[] patMsg  = new byte[MSG_SIZE];
        PutU32(maskMsg, OFF_PROTO, ISO9141);
        PutU32(patMsg,  OFF_PROTO, ISO9141);
        PutU32(maskMsg, OFF_DSIZE, 1);   // DataSize=1, mask byte 0x00 (arrays zero-init) → match all
        PutU32(patMsg,  OFF_DSIZE, 1);   // pattern byte 0x00 → everything passes
        GCHandle hMask = GCHandle.Alloc(maskMsg, GCHandleType.Pinned);
        GCHandle hPat  = GCHandle.Alloc(patMsg,  GCHandleType.Pinned);
        uint filtId;
        r = StartFilter(chanId, PASS_FILTER, hMask.AddrOfPinnedObject(),
                    hPat.AddrOfPinnedObject(), IntPtr.Zero, out filtId);
        Check(r, "StartMsgFilter (pass-all)");
        hMask.Free(); hPat.Free();

        Thread.Sleep(100);

        // Brake-watch mode: sample the brake-switch byte repeatedly so the bit can be
        // seen toggling live. Run as:  SsmInitTiming.exe brake
        if (brakeMode)
        {
            Log("══ Brake-switch watch — press/release the pedal (S67 bit3, S64 bit6) ══");
            for (int i = 0; i < 40; i++)
            {
                byte[] d = ReadAddrs(chanId, new byte[][] { ParseAddr("0x000121") });
                if (d != null)
                    Log(string.Format("  [{0,2}]  byte=0x{1:X2}   Brake(S67)={2}   StopLight(S64)={3}",
                        i, d[0], (d[0] >> 3) & 1, (d[0] >> 6) & 1));
                else
                    Log("  [" + i + "]  no response");
                Thread.Sleep(200);
            }
            Disconnect(chanId); Close(devId);
            Log("\nDone (brake watch).");
            return;
        }

        // ════════════════════════════════════════════════════════════════════
        // TEST 1 — SSM Init (0xBF) repeated INIT_ITERS times
        //
        // Packet:   80 10 F0 01 BF <cs>  (6 bytes)
        // Response: 80 F0 10 <len> E0 <5-byte ROM ID> <capabilities...> <cs>
        //
        // Round-trip breakdown:
        //   TX time      = 6 bytes × 2.083 ms = 12.5 ms  (fixed)
        //   Echo time    = 6 bytes × 2.083 ms = 12.5 ms  (fixed, half-duplex loopback)
        //   ECU gap      = measured - TX - echo - resp_rx  ← what we want
        //   Resp RX time = resp_len × 2.083 ms            (fixed, varies by ECU capabilities)
        //
        // ECU gap → Phase 1 first-byte timeout = ECU gap + 20 ms margin
        // ════════════════════════════════════════════════════════════════════
        Log("══ Test 1: SSM Init (0xBF) — " + INIT_ITERS + " iterations ══════════════════════════");

        byte[] initPkt = BuildInit();
        double initTxMs = KLineMs(initPkt.Length);
        Log("TX packet: " + Hex(initPkt) + "  (" + initPkt.Length + " bytes, " + initTxMs.ToString("F1") + " ms)");
        Log("");

        long[] initDeltas  = new long[INIT_ITERS];
        int[]  initRespLen = new int[INIT_ITERS];
        int    initGood    = 0;
        int    consecTO    = 0;   // consecutive timeouts → bail out early
        string romId       = null;
        byte[] initResp    = null;   // full response, kept for the pre-flight decode below

        for (int i = 0; i < INIT_ITERS; i++)
        {
            if (i > 0) Thread.Sleep(250); // let ECU settle between inits

            long ta = Stopwatch.GetTimestamp();
            Send(chanId, initPkt);
            byte[] resp = Recv(chanId, 3000);
            long tb = Stopwatch.GetTimestamp();

            if (resp == null)
            {
                Log("  [" + i + "] TIMEOUT — check key-on / wiring");
                if (++consecTO >= 3) { Log(""); Log("3 consecutive timeouts — aborting (no ECU response)."); break; }
                continue;
            }
            consecTO = 0;

            initDeltas[initGood]  = tb - ta;
            initRespLen[initGood] = resp.Length;
            initGood++;

            // ROM ID = response bytes [8..12] (after 0xFF + 3-byte SSM-ID + ).  This is the
            // value the firmware, RomRaider, and the defs use to match <ecuparam> blocks.
            if (romId == null && resp.Length >= 13)
            {
                initResp = resp;
                var sb = new StringBuilder();
                for (int b = 8; b <= 12; b++) sb.AppendFormat("{0:X2}", resp[b]);
                romId = sb.ToString();
            }

            double dtMs = Ticks2Ms(tb - ta);
            if (i == 0)
                Log("  [" + i + "] " + resp.Length + " bytes  dt=" + dtMs.ToString("F2") + " ms  RX: " + Hex(resp));
            else
                Log("  [" + i + "] " + resp.Length + " bytes  dt=" + dtMs.ToString("F2") + " ms");
        }

        Log("");
        if (initGood == 0) {
            Log("No valid init responses — aborting.");
            Disconnect(chanId); Close(devId); return;
        }

        // Stats
        long iSum = 0, iMin = long.MaxValue, iMax = 0;
        int  rSum = 0;
        for (int i = 0; i < initGood; i++) {
            iSum += initDeltas[i];
            rSum += initRespLen[i];
            if (initDeltas[i] < iMin) iMin = initDeltas[i];
            if (initDeltas[i] > iMax) iMax = initDeltas[i];
        }
        double iMeanMs   = Ticks2Ms(iSum / initGood);
        double iMinMs    = Ticks2Ms(iMin);
        double iMaxMs    = Ticks2Ms(iMax);
        int    meanRLen  = rSum / initGood;
        double respRxMs  = KLineMs(meanRLen);
        // Half-duplex K-line: our request and its loopback echo occupy the SAME wire
        // window, so round-trip = TX + ECU-gap + response. (Echo is NOT additive.)
        double initGapMs   = iMeanMs - initTxMs - respRxMs;
        double initRetryMs = iMaxMs + 15;   // full round-trip drained + margin

        Log("── Init timing results (" + initGood + "/" + INIT_ITERS + " OK) ──────────────────────────────────");
        Log("  ECU ROM ID:        " + (romId ?? "unknown"));
        Log("  Response length:   " + meanRLen + " bytes  (" + respRxMs.ToString("F1") + " ms receive time)");
        Log("");
        Log("  Round-trip  min:   " + iMinMs.ToString("F2")  + " ms");
        Log("  Round-trip  mean:  " + iMeanMs.ToString("F2") + " ms");
        Log("  Round-trip  max:   " + iMaxMs.ToString("F2")  + " ms");
        Log("");
        Log("  Breakdown (mean):  TX + ECU-gap + response = round-trip");
        Log("    TX request (6 bytes):  " + initTxMs.ToString("F1") + " ms  (on wire; our echo is read back during this)");
        Log("    ECU proc gap:          " + initGapMs.ToString("F1")+ " ms  (end of TX to first response byte)");
        Log("    Response receive:      " + respRxMs.ToString("F1") + " ms  (" + meanRLen + " bytes)");
        Log("");
        Log("  ► Phase 1 first-byte timeout = " + initGapMs.ToString("F0") + " (gap) + 20 ms margin = " + (initGapMs + 20).ToString("F0") + " ms");
        Log("    (max wait for the ECU's first response byte after sending)");
        Log("  ► Minimum safe retry interval = " + iMaxMs.ToString("F0") + " (max round-trip) + 15 ms = " + initRetryMs.ToString("F0") + " ms");
        Log("    (wait this long before re-sending 0xBF if mid-response timeout suspected)");

        // ════════════════════════════════════════════════════════════════════
        // PRE-FLIGHT CHECKS — validate what the Arduino firmware depends on
        // ════════════════════════════════════════════════════════════════════
        if (initResp != null)
        {
            string defs = FindDefs();
            DecodeInit(initResp);                          // ROM ID + capability bitmap + checksum check
            MapCapabilities(initResp, defs);               // capability bits → supported P-param names
            DefaultParamCheck(chanId, defs, initResp);     // the 12 default params at their real addresses
            ExtendedAddrCheck(chanId, defs, romId);        // E-param extended-address reads
            SwitchCheck(chanId, defs, initResp);           // switch (S-param) T_BIT reads
        }
        LiveParamChecks(chanId);                  // read known params, decode engineering values
        MaxBatchProbe(chanId);                    // largest batch the ECU answers

        // ════════════════════════════════════════════════════════════════════
        // TEST 2 — Batch read (0xA8, slow poll) repeated READ_ITERS times
        //
        // Packet: 80 10 F0 <len> A8 00 <addrs...> <cs>
        // Response: 80 F0 10 <len> E8 <data bytes> <cs>
        //
        // ECU gap here → Phase 2 poll gap / watchdog timeout reference
        // ════════════════════════════════════════════════════════════════════
        Log("");
        Log("══ Test 2: Batch read " + TEST_ADDRS.Length + " address(es) — " + READ_ITERS + " iterations ══════════════════════");

        byte[] readPkt     = BuildBatchRead(TEST_ADDRS, continuous: false);
        int    expRespLen  = 6 + TEST_ADDRS.Length;
        double readTxMs    = KLineMs(readPkt.Length);
        double readRxMs    = KLineMs(expRespLen);

        Log("TX packet: " + Hex(readPkt) + "  (" + readPkt.Length + " bytes, " + readTxMs.ToString("F1") + " ms)");
        Log("Expected response: " + expRespLen + " bytes  (" + readRxMs.ToString("F1") + " ms)");
        Log("");

        long[] readDeltas = new long[READ_ITERS];
        int    readGood   = 0;

        for (int i = 0; i < READ_ITERS; i++)
        {
            long ta = Stopwatch.GetTimestamp();
            Send(chanId, readPkt);
            byte[] resp = Recv(chanId, 2000);
            long tb = Stopwatch.GetTimestamp();

            if (resp != null)
            {
                readDeltas[readGood++] = tb - ta;
                if (i == 0)
                    Log("  [" + i + "] dt=" + Ticks2Ms(tb-ta).ToString("F2") + " ms  RX: " + Hex(resp));
                else
                    Log("  [" + i + "] dt=" + Ticks2Ms(tb-ta).ToString("F2") + " ms");
            }
            else
            {
                Log("  [" + i + "] TIMEOUT");
            }
            Thread.Sleep(20);
        }

        Log("");
        if (readGood > 0)
        {
            long rSum2 = 0, rMin = long.MaxValue, rMax = 0;
            for (int i = 0; i < readGood; i++) {
                rSum2 += readDeltas[i];
                if (readDeltas[i] < rMin) rMin = readDeltas[i];
                if (readDeltas[i] > rMax) rMax = readDeltas[i];
            }
            double rMeanMs = Ticks2Ms(rSum2 / readGood);
            double rMinMs  = Ticks2Ms(rMin);
            double rMaxMs  = Ticks2Ms(rMax);
            double readGapMs  = rMeanMs - readTxMs - readRxMs;   // echo concurrent with TX (half-duplex)

            Log("── Batch read results (" + readGood + "/" + READ_ITERS + " OK) ─────────────────────────────────");
            Log("  Round-trip  min:   " + rMinMs.ToString("F2")  + " ms");
            Log("  Round-trip  mean:  " + rMeanMs.ToString("F2") + " ms");
            Log("  Round-trip  max:   " + rMaxMs.ToString("F2")  + " ms");
            Log("");
            Log("  Breakdown (mean):  TX + ECU-gap + response = round-trip");
            Log("    TX request (" + readPkt.Length + " bytes):  " + readTxMs.ToString("F1") + " ms  (echo read back during this)");
            Log("    ECU proc gap:          " + readGapMs.ToString("F1") + " ms  (end of TX to first response byte)");
            Log("    Response receive:      " + readRxMs.ToString("F1")  + " ms  (" + expRespLen + " bytes)");
            Log("");
            Log("  ► Max slow-poll rate (no fast-poll): ~" + (1000.0 / rMeanMs).ToString("F0") + " Hz");
            Log("  ► Phase 2 watchdog timeout: " + (rMaxMs * 3).ToString("F0") + " ms  (3× max round-trip)");
        }

        ContinuousTest(chanId);   // item 4 — streaming fast-poll rate (run last; it leaves the bus quiet)

        // ════════════════════════════════════════════════════════════════════
        // SUMMARY — Copy these values into Arduino firmware
        // ════════════════════════════════════════════════════════════════════
        Log("");
        Log("════════════════════════════════════════════════════════════════");
        Log("FIRMWARE CONSTANTS (copy into Arduino sketch)");
        Log("════════════════════════════════════════════════════════════════");
        Log("  ECU ROM ID:                 " + (romId ?? "unknown"));
        Log("  Phase 1 first-byte timeout: " + (initGapMs + 20).ToString("F0") + " ms");
        Log("  Min safe retry interval:    " + initRetryMs.ToString("F0") + " ms");
        if (readGood > 0)
        {
            long rSum2 = 0; for (int i = 0; i < readGood; i++) rSum2 += readDeltas[i];
            double rMeanMs = Ticks2Ms(rSum2 / readGood);
            long   rMax    = 0; for (int i = 0; i < readGood; i++) if (readDeltas[i] > rMax) rMax = readDeltas[i];
            Log("  Phase 2 watchdog timeout:   " + (Ticks2Ms(rMax) * 3).ToString("F0") + " ms");
        }
        Log("════════════════════════════════════════════════════════════════");

        Disconnect(chanId);
        Close(devId);
        Log("\nDone.");
    }

    // ── Pre-flight checks ────────────────────────────────────────────────────

    // Build a 3-byte SSM address 0x0000xx (low RAM, where the standard P-params live).
    static byte[] Addr(int low) { return new byte[] { 0x00, 0x00, (byte)low }; }

    // Single 0xA8 batch read of the given addresses; returns the N data bytes, or null.
    static byte[] ReadAddrs(uint chanId, byte[][] addrs)
    {
        Thread.Sleep(10);
        Send(chanId, BuildBatchRead(addrs, false));
        byte[] r = Recv(chanId, 2500);
        // Response: 80 F0 10 <N+1> E8 <N data bytes> <cs>
        if (r == null || r.Length < 6 + addrs.Length) return null;
        if (r[0] != 0x80 || r[1] != 0xF0 || r[2] != 0x10 || r[4] != 0xE8) return null;
        byte[] data = new byte[addrs.Length];
        Array.Copy(r, 5, data, 0, addrs.Length);
        return data;
    }

    static string Hex2(byte[] b, int off, int len)
    {
        var sb = new StringBuilder();
        for (int i = 0; i < len && off + i < b.Length; i++) { if (i > 0) sb.Append(' '); sb.Append(b[off + i].ToString("X2")); }
        return sb.ToString();
    }

    // Decode the SSM init response into labeled sections + the capability bitmap.
    static void DecodeInit(byte[] r)
    {
        Log("");
        Log("── ECU init response decode ─────────────────────────────────────────");
        if (r.Length < 14) { Log("    (response too short to decode)"); return; }
        int dataLen  = r[3];
        int capStart = 13;             // after 0xFF(4) + SSM-ID(5..7) + ROM-ID(8..12)
        int capEnd   = 4 + dataLen;    // index of the checksum byte
        if (capEnd > r.Length - 1) capEnd = r.Length - 1;
        Log("    Header:          " + Hex2(r, 0, 4) + "   (80 F0 10 len)");
        Log("    Command:         " + r[4].ToString("X2"));
        Log("    SSM ID:          " + Hex2(r, 5, 3));
        Log("    ROM ID:          " + Hex2(r, 8, 5) + "   <- matches firmware / defs <ecuparam>");
        Log("    Capability (" + (capEnd - capStart) + " B): " + Hex2(r, capStart, capEnd - capStart));
        byte cs = 0; for (int k = 0; k < r.Length - 1; k++) cs += r[k];
        Log("    Checksum:        " + r[r.Length - 1].ToString("X2") +
            (cs == r[r.Length - 1] ? "   (verified: matches our SSM checksum calc)"
                                   : "   (MISMATCH! we calc " + cs.ToString("X2") + ")"));

        int setCount = 0;
        for (int i = capStart; i < capEnd; i++)
            for (int bit = 0; bit < 8; bit++)
                if ((r[i] & (1 << bit)) != 0) setCount++;
        Log("    Capability bits set: " + setCount + "  (each gates one P-param via rawResp[5+byteIndex] & (1<<bit))");
    }

    // Parse a logger_*.xml and list which capability-gated P-params THIS ECU supports.
    static void MapCapabilities(byte[] r, string defsPath)
    {
        Log("");
        Log("── Supported P-parameters (capability bit -> name via defs) ──────────");
        if (defsPath == null || !File.Exists(defsPath))
        {
            Log("    (no logger_*.xml in this folder — skipping name mapping)");
            return;
        }
        Log("    defs: " + Path.GetFileName(defsPath));
        int supported = 0, gated = 0;
        foreach (string line in File.ReadAllLines(defsPath))
        {
            if (line.IndexOf("</protocol>") >= 0) break;   // SSM is the first protocol; stop before OBD
            if (line.IndexOf("<parameter ") < 0) continue;
            string bi = Attr(line, "ecubyteindex");
            string bt = Attr(line, "ecubit");
            if (bi == null || bt == null) continue;
            int byteIdx, bit;
            if (!int.TryParse(bi, out byteIdx) || !int.TryParse(bt, out bit)) continue;
            gated++;
            int idx = 5 + byteIdx;     // firmware: rawResp[5 + ecuByteIndex]
            if (idx < r.Length && (r[idx] & (1 << bit)) != 0)
            {
                supported++;
                Log(string.Format("    {0,-5} {1}", Attr(line, "id"), Attr(line, "name")));
            }
        }
        Log("    -> " + supported + " of " + gated + " capability-gated P-params supported by this ECU.");
    }

    // Live-read a few well-known params at fixed SSM addresses and decode them.
    static void LiveParamChecks(uint chanId)
    {
        Log("");
        Log("── Live parameter sanity reads ──────────────────────────────────────");
        Log("  (proves read -> byte-order -> scaling end-to-end; values should look sane)");
        byte[] d;
        d = ReadAddrs(chanId, new[] { Addr(0x0E), Addr(0x0F) });
        if (d != null) Log(string.Format("    Engine speed:    {0,7:F0} rpm    (raw {1:X2}{2:X2})", (d[0] * 256 + d[1]) / 4.0, d[0], d[1]));
        d = ReadAddrs(chanId, new[] { Addr(0x08) });
        if (d != null) Log(string.Format("    Coolant temp:    {0,7:F0} C      (raw {1:X2})", d[0] - 40.0, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x12) });
        if (d != null) Log(string.Format("    Intake air temp: {0,7:F0} C      (raw {1:X2})", d[0] - 40.0, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x15) });
        if (d != null) Log(string.Format("    Throttle:        {0,7:F1} %      (raw {1:X2})", d[0] * 100.0 / 255.0, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x10) });
        if (d != null) Log(string.Format("    Vehicle speed:   {0,7:F0} km/h   (raw {1:X2})", (double)d[0], d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x1C) });
        if (d != null) Log(string.Format("    Battery:         {0,7:F2} V      (raw {1:X2})", d[0] * 0.08, d[0]));
    }

    // Probe how large a single 0xA8 batch this ECU will answer (firmware budget ~84).
    static void MaxBatchProbe(uint chanId)
    {
        Log("");
        Log("── Max batch-read probe ─────────────────────────────────────────────");
        int[] sizes = { 1, 8, 16, 24, 32, 34, 36, 38, 40, 44, 48, 56, 64, 80, 84 };
        int lastOk = 0;
        foreach (int n in sizes)
        {
            var addrs = new byte[n][];
            for (int i = 0; i < n; i++) addrs[i] = new byte[] { 0x00, 0x00, (byte)i };
            Send(chanId, BuildBatchRead(addrs, false));
            byte[] resp = Recv(chanId, 2500);
            bool ok = resp != null && resp.Length >= 6 + n && resp[4] == 0xE8;
            Log(string.Format("    {0,3} addr -> {1}", n, ok ? "OK (" + resp.Length + " bytes)" : "no/bad response"));
            if (ok) lastOk = n; else break;
            Thread.Sleep(30);
        }
        Log("    -> ECU answered batches up to " + lastOk + " addresses.");
    }

    // Extract attr="value" from a line; null if absent.
    static string Attr(string line, string name)
    {
        int i = line.IndexOf(name + "=\"");
        if (i < 0) return null;
        i += name.Length + 2;
        int j = line.IndexOf('"', i);
        return j < 0 ? null : line.Substring(i, j - i);
    }

    // Pick the defs file for capability name mapping — prefer the firmware's primary
    // (logger_IMP_EN_v370.xml), then any English variant, then the first logger_*.xml.
    static string FindDefs()
    {
        try {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            string primary = Path.Combine(dir, "logger_IMP_EN_v370.xml");
            if (File.Exists(primary)) return primary;
            var files = Directory.GetFiles(dir, "logger_*.xml");
            foreach (var f in files) if (f.IndexOf("_EN_", StringComparison.OrdinalIgnoreCase) >= 0) return f;
            return files.Length > 0 ? files[0] : null;
        } catch { return null; }
    }

    // Build a 3-byte SSM address from a "0xNNNNNN" hex string.
    static byte[] ParseAddr(string hex)
    {
        hex = hex.Trim();
        if (hex.StartsWith("0x") || hex.StartsWith("0X")) hex = hex.Substring(2);
        uint v = Convert.ToUInt32(hex, 16);
        return new byte[] { (byte)(v >> 16), (byte)(v >> 8), (byte)v };
    }

    // N consecutive 3-byte addresses from a base (for multi-byte params).
    static byte[][] ConsecAddrs(byte[] b, int n)
    {
        uint v = (uint)((b[0] << 16) | (b[1] << 8) | b[2]);
        var res = new byte[n][];
        for (int i = 0; i < n; i++) { uint a = v + (uint)i; res[i] = new byte[] { (byte)(a >> 16), (byte)(a >> 8), (byte)a }; }
        return res;
    }

    static string Trunc(string s, int n) { if (s == null) return ""; return s.Length <= n ? s : s.Substring(0, n); }

    // ITEM 1 — read real ECU-specific (E-param) addresses for THIS ROM. They live in
    // the extended 0x02xxxx / 0xFFxxxx region (everything else we read is low RAM), so
    // this confirms the ECU answers extended addresses — what E-params + immo logging need.
    static void ExtendedAddrCheck(uint chanId, string defsPath, string romId)
    {
        Log("");
        Log("── Extended-address (E-param) read test ─────────────────────────────");
        Log("  (E-params + immo registers live at 0x02xxxx/0xFFxxxx — confirm those answer)");
        if (defsPath == null || !File.Exists(defsPath)) { Log("    (no defs — skipping)"); return; }
        string[] lines = File.ReadAllLines(defsPath);
        string name = null; bool wantAddr = false; int tested = 0, ok = 0;
        for (int i = 0; i < lines.Length && tested < 4; i++)
        {
            string line = lines[i];
            if (line.IndexOf("<ecuparam ") >= 0) { name = Attr(line, "name"); wantAddr = false; continue; }
            if (line.IndexOf("<ecu ") >= 0) { string id = Attr(line, "id"); wantAddr = (id != null && id.IndexOf(romId) >= 0); continue; }
            if (wantAddr && line.IndexOf("<address") >= 0)
            {
                wantAddr = false;
                string lenS = Attr(line, "length"); int len = 1; if (lenS != null) int.TryParse(lenS, out len);
                int gt = line.IndexOf('>', line.IndexOf("<address"));
                int lt = gt >= 0 ? line.IndexOf('<', gt + 1) : -1;
                if (gt < 0 || lt <= gt) continue;
                string addrHex = line.Substring(gt + 1, lt - gt - 1).Trim();
                byte[] baseA; try { baseA = ParseAddr(addrHex); } catch { continue; }
                tested++;
                byte[] data = ReadAddrs(chanId, ConsecAddrs(baseA, len));
                if (data != null) ok++;
                Log(string.Format("    {0,-34} @ {1} (len {2}) -> {3}",
                    Trunc(name, 34), addrHex, len, data != null ? "OK raw " + Hex(data) : "NO RESPONSE"));
            }
        }
        if (tested == 0) Log("    (no ecuparam addresses found for ROM " + romId + ")");
        else Log("    -> " + ok + "/" + tested + " extended-address reads succeeded.");
    }

    // ITEM 3 — read the firmware's 12 default-profile params at their real defs addresses.
    class PInfo { public string name, addrHex; public int len = 1, byteIdx, bit; public bool hasCap; }

    static PInfo LookupParam(string[] lines, string id)
    {
        PInfo pi = null; bool inParam = false;
        foreach (string line in lines)
        {
            if (line.IndexOf("</protocol>") >= 0) break;          // SSM section only
            if (!inParam)
            {
                if (line.IndexOf("<parameter ") >= 0 && Attr(line, "id") == id)
                {
                    pi = new PInfo(); pi.name = Attr(line, "name");
                    string bi = Attr(line, "ecubyteindex"), bt = Attr(line, "ecubit");
                    pi.hasCap = (bi != null && bt != null && int.TryParse(bi, out pi.byteIdx) && int.TryParse(bt, out pi.bit));
                    inParam = true;
                }
            }
            else
            {
                if (pi.addrHex == null && line.IndexOf("<address") >= 0)
                {
                    string lenS = Attr(line, "length"); if (lenS != null) int.TryParse(lenS, out pi.len);
                    int gt = line.IndexOf('>', line.IndexOf("<address"));
                    int lt = gt >= 0 ? line.IndexOf('<', gt + 1) : -1;
                    if (gt >= 0 && lt > gt) pi.addrHex = line.Substring(gt + 1, lt - gt - 1).Trim();
                }
                if (line.IndexOf("</parameter>") >= 0) break;
            }
        }
        return pi;
    }

    static void DefaultParamCheck(uint chanId, string defsPath, byte[] initResp)
    {
        Log("");
        Log("── Default-profile params (firmware's 12 enabled-by-default) ─────────");
        if (defsPath == null || !File.Exists(defsPath)) { Log("    (no defs — skipping)"); return; }
        string[] lines = File.ReadAllLines(defsPath);
        string[] basic = { "P8", "P2", "P9", "P7", "P11", "P10", "P12", "P13", "P3", "P4", "P14", "P17" };
        int sup = 0, okRead = 0;
        foreach (string id in basic)
        {
            PInfo pi = LookupParam(lines, id);
            if (pi == null) { Log(string.Format("    {0,-5} (not in defs)", id)); continue; }
            bool supported = !pi.hasCap || (initResp != null && (5 + pi.byteIdx) < initResp.Length && (initResp[5 + pi.byteIdx] & (1 << pi.bit)) != 0);
            string status;
            if (!supported) status = "NOT supported by ECU (firmware skips it)";
            else
            {
                sup++;
                if (pi.addrHex == null) status = "supported (no SSM address)";
                else
                {
                    byte[] data = ReadAddrs(chanId, ConsecAddrs(ParseAddr(pi.addrHex), pi.len));
                    if (data != null) { okRead++; status = "OK  @ " + pi.addrHex + " raw " + Hex(data); }
                    else status = "@ " + pi.addrHex + " -> NO RESPONSE";
                }
            }
            Log(string.Format("    {0,-5} {1,-26} {2}", id, Trunc(pi.name, 26), status));
        }
        Log("    -> " + sup + "/12 supported, " + okRead + " read OK.");
    }

    // Switch (S-param) read — the T_BIT logging path: read the byte= address, extract bit=.
    static void SwitchCheck(uint chanId, string defsPath, byte[] initResp)
    {
        Log("");
        Log("── Switch (S-param) read test ───────────────────────────────────────");
        Log("  (bit-flags the firmware logs via T_BIT — validates that path)");
        if (defsPath == null || !File.Exists(defsPath)) { Log("    (no defs — skipping)"); return; }
        string[] lines = File.ReadAllLines(defsPath);
        int shown = 0, ok = 0;
        foreach (string line in lines)
        {
            if (line.IndexOf("</protocol>") >= 0) break;       // SSM section only
            if (line.IndexOf("<switch ") < 0) continue;
            if (shown >= 12) break;
            string addrS = Attr(line, "byte"), bitS = Attr(line, "bit"), biS = Attr(line, "ecubyteindex");
            if (addrS == null || bitS == null) continue;
            int bit; if (!int.TryParse(bitS, out bit)) continue;
            int bi = -1; if (biS != null) int.TryParse(biS, out bi);
            bool sup = bi < 0 || (initResp != null && (5 + bi) < initResp.Length && (initResp[5 + bi] & (1 << bit)) != 0);
            if (!sup) continue;
            shown++;
            byte[] d; try { d = ReadAddrs(chanId, new byte[][] { ParseAddr(addrS) }); } catch { continue; }
            if (d == null) { Log(string.Format("    {0,-5} {1,-30} read FAIL @ {2}", Attr(line, "id"), Trunc(Attr(line, "name"), 30), addrS)); continue; }
            ok++;
            int state = (d[0] >> bit) & 1;
            Log(string.Format("    {0,-5} {1,-30} = {2}   ({3} bit {4}, byte {5:X2})",
                Attr(line, "id"), Trunc(Attr(line, "name"), 30), state, addrS, bit, d[0]));
        }
        if (shown == 0) Log("    (no supported switches found for this ECU)");
        else Log("    -> " + ok + "/" + shown + " switches read OK — T_BIT path validated.");
    }

    // ITEM 4 — continuous fast-poll: ECU streams 0xA8 responses without re-request.
    static void ContinuousTest(uint chanId)
    {
        Log("");
        Log("── Continuous fast-poll mode (0xA8 flag=1) ──────────────────────────");
        Log("  (ECU streams without re-request; measures the max achievable sample rate)");
        byte[][] addrs = { Addr(0x0E), Addr(0x0F), Addr(0x08) };   // RPM (2 bytes) + coolant
        Send(chanId, BuildBatchRead(addrs, true));
        int got = 0; long prev = 0; double sum = 0, min = 1e9, max = 0;
        for (int i = 0; i < 12; i++)
        {
            byte[] r = Recv(chanId, 1000);
            long t = Stopwatch.GetTimestamp();
            if (r == null) break;
            if (got > 0) { double dt = Ticks2Ms(t - prev); sum += dt; if (dt < min) min = dt; if (dt > max) max = dt; }
            prev = t; got++;
        }
        // Stop the stream: a single (flag=0) read returns the ECU to single-response mode.
        Send(chanId, BuildBatchRead(addrs, false)); Recv(chanId, 1000);
        if (got >= 3)
        {
            double mean = sum / (got - 1);
            Log(string.Format("    streamed {0} responses; period mean {1:F1} ms (min {2:F1}, max {3:F1})", got, mean, min, max));
            Log(string.Format("    -> continuous rate ~{0:F0} Hz (vs ~17 Hz single-poll) for {1} addrs", 1000.0 / mean, addrs.Length));
        }
        else Log("    ECU did not stream — continuous mode unsupported here; single-poll still works.");
    }

    // ── SSM packet builders ──────────────────────────────────────────────────

    // Init: 80 10 F0 01 BF <cs>
    static byte[] BuildInit()
    {
        byte[] pkt = new byte[6];
        pkt[0] = 0x80; pkt[1] = 0x10; pkt[2] = 0xF0; pkt[3] = 0x01; pkt[4] = 0xBF;
        pkt[5] = Checksum(pkt, 5);
        return pkt;
    }

    // Batch read: 80 10 F0 <len> A8 <flag> <addrs...> <cs>
    static byte[] BuildBatchRead(byte[][] addrs, bool continuous)
    {
        int payloadLen = 1 + 1 + 3 * addrs.Length; // A8 + flag + addrs
        byte[] pkt = new byte[4 + payloadLen + 1];
        pkt[0] = 0x80; pkt[1] = 0x10; pkt[2] = 0xF0; pkt[3] = (byte)payloadLen;
        pkt[4] = 0xA8;
        pkt[5] = (byte)(continuous ? 1 : 0);
        int pos = 6;
        foreach (var a in addrs) { pkt[pos++] = a[0]; pkt[pos++] = a[1]; pkt[pos++] = a[2]; }
        pkt[pos] = Checksum(pkt, pos);
        return pkt;
    }

    static byte Checksum(byte[] pkt, int csPos)
    {
        uint s = 0;
        for (int i = 0; i < csPos; i++) s += pkt[i];   // include the 0x80 header byte
        return (byte)(s & 0xFF);
    }

    // ── J2534 send/receive ───────────────────────────────────────────────────

    static void Send(uint chanId, byte[] data)
    {
        byte[] msg = new byte[MSG_SIZE];
        PutU32(msg, OFF_PROTO, ISO9141);
        PutU32(msg, OFF_DSIZE, (uint)data.Length);
        Array.Copy(data, 0, msg, OFF_DATA, data.Length);
        GCHandle h = GCHandle.Alloc(msg, GCHandleType.Pinned);
        uint num = 1;
        int r = WriteMsgs(chanId, h.AddrOfPinnedObject(), ref num, 500);
        h.Free();
        if (Verbose)
        {
            var e = new StringBuilder(256); GetLastError(e);
            Log(string.Format("    [tx] WriteMsgs r={0} num={1} err={2}", r, num, e));
        }
    }

    static bool Verbose = false;   // set true to dump every message ReadMsgs returns

    // Returns first non-echo message within timeoutMs, or null
    static byte[] Recv(uint chanId, uint timeoutMs)
    {
        byte[] msg = new byte[MSG_SIZE];
        GCHandle h = GCHandle.Alloc(msg, GCHandleType.Pinned);
        long deadline = Stopwatch.GetTimestamp() +
                        (long)(timeoutMs * (double)Stopwatch.Frequency / 1000.0);
        byte[] result = null;
        int msgsSeen = 0, echoSeen = 0, lastR = -999;
        while (Stopwatch.GetTimestamp() < deadline)
        {
            uint num = 1;
            int r = ReadMsgs(chanId, h.AddrOfPinnedObject(), ref num, 50);
            lastR = r;
            if (r == 0 && num > 0)
            {
                msgsSeen++;
                uint rxSt  = GetU32(msg, OFF_RXST);
                uint dsize = GetU32(msg, OFF_DSIZE);
                if (rxSt != 0) echoSeen++;   // loopback(0x01)/TX(0x08)/start-of-msg(0x02) — not data
                if (Verbose)
                {
                    int n = (int)Math.Min(dsize, 24);
                    byte[] tmp = new byte[n < 0 ? 0 : n];
                    if (tmp.Length > 0) Array.Copy(msg, OFF_DATA, tmp, 0, tmp.Length);
                    Log(string.Format("    [rx] r=0 num={0} rxSt=0x{1:X} dsize={2} data={3}",
                                      num, rxSt, dsize, Hex(tmp)));
                }
                if (rxSt == 0 && dsize > 0)   // RX_INDICATION with data = the ECU response
                {
                    result = new byte[dsize];
                    Array.Copy(msg, OFF_DATA, result, 0, (int)dsize);
                    break;
                }
            }
        }
        if (Verbose && result == null)
        {
            var e = new StringBuilder(256); GetLastError(e);
            Log(string.Format("    [rx] TIMEOUT  msgsSeen={0} (echoes={1})  lastReadMsgs_r={2}  lastErr={3}",
                              msgsSeen, echoSeen, lastR, e));
        }
        h.Free();
        return result;
    }

    // ── Utilities ────────────────────────────────────────────────────────────

    static double Ticks2Ms(long t) { return t * 1000.0 / Stopwatch.Frequency; }
    static double KLineMs(int bytes) { return bytes * 10.0 / 4800.0 * 1000.0; }

    static void PutU32(byte[] b, int o, uint v) {
        b[o]=(byte)v; b[o+1]=(byte)(v>>8); b[o+2]=(byte)(v>>16); b[o+3]=(byte)(v>>24);
    }
    static uint GetU32(byte[] b, int o) {
        return (uint)(b[o] | (b[o+1]<<8) | (b[o+2]<<16) | (b[o+3]<<24));
    }
    static string Hex(byte[] b) { return BitConverter.ToString(b).Replace("-"," "); }

    static bool Check(int r, string label) {
        if (r == 0) { Log(label + ": OK"); return true; }
        var e = new StringBuilder(256); GetLastError(e);
        Log(label + " FAILED r=" + r + " — " + e);
        return false;
    }
    static void Log(string s) { Console.WriteLine(s); }
    static T Fn<T>(IntPtr h, string name) where T : class {
        return Marshal.GetDelegateForFunctionPointer(GetProcAddress(h, name), typeof(T)) as T;
    }
}
