// SsmInitTiming.cs — Measure SSM init (0xBF) and batch read latency via J2534 (Tactrix OpenPort 2.0)
//
// Produces two sets of timing numbers needed for Arduino K-Line logger firmware:
//   Test 1: SSM init (0xBF) — 15 iterations
//           → ECU ROM ID, init ECU proc gap, recommended Phase 1 first-byte timeout
//   Test 2: Batch read (0xA8, slow poll) — 20 iterations
//           → read ECU proc gap, recommended Phase 2 fast-poll watchdog
//
// Compile: C:\Windows\Microsoft.NET\Framework\v4.0.30319\csc.exe /platform:x86 SsmInitTiming.cs /out:SsmInitTiming.exe
// Run:     SsmInitTiming.exe
// Prereq:  Tactrix plugged into OBD2, key-on or engine running, RomRaider NOT open

using System;
using System.Diagnostics;
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

    const int INIT_ITERS = 15;
    const int READ_ITERS = 20;

    // ── J2534 function pointers ──────────────────────────────────────────────
    static dWriteMsgs   WriteMsgs;
    static dReadMsgs    ReadMsgs;
    static dGetLastError GetLastError;

    // ────────────────────────────────────────────────────────────────────────
    static void Main()
    {
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

        for (int i = 0; i < INIT_ITERS; i++)
        {
            if (i > 0) Thread.Sleep(250); // let ECU settle between inits

            Verbose = (i == 0);   // dump device-level detail on the first attempt
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

            // Extract ROM ID from first successful response: bytes [5..9]
            if (romId == null && resp.Length >= 10)
            {
                var sb = new StringBuilder();
                for (int b = 5; b <= 9; b++) sb.AppendFormat("{0:X2}", resp[b]);
                romId = sb.ToString();
            }

            double dtMs = Ticks2Ms(tb - ta);
            if (i < 3)
                Log("  [" + i + "] " + resp.Length + " bytes  dt=" + dtMs.ToString("F2") + " ms  RX: " + Hex(resp));
            else if (i == 3)
                Log("  (suppressing raw dumps for remaining iterations)");
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
                if (i < 3)
                    Log("  [" + i + "] dt=" + Ticks2Ms(tb-ta).ToString("F2") + " ms  RX: " + Hex(resp));
                else if (i == 3)
                    Log("  (suppressing raw dumps)");
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
