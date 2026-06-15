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
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using Microsoft.Win32;
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

    // ── Transport: J2534 (Tactrix) or generic USB-serial K-line cable ─────────
    static bool       g_serial = false;   // false = J2534, true = serial COM port
    static SerialPort g_port   = null;    // the serial cable (when g_serial)
    static int        g_maxBatch = 32;    // ECU's probed max batch-read size (set by MaxBatchProbe)

    // Find the registered J2534 driver DLL via the registry (so it works on any
    // machine with a Tactrix driver, wherever installed); fall back to the hardcoded path.
    static string FindJ2534Dll()
    {
        string[] roots = {
            @"SOFTWARE\WOW6432Node\PassThruSupport.04.04",
            @"SOFTWARE\PassThruSupport.04.04"
        };
        foreach (string root in roots)
        {
            try {
                using (RegistryKey k = Registry.LocalMachine.OpenSubKey(root))
                {
                    if (k == null) continue;
                    foreach (string sub in k.GetSubKeyNames())
                    {
                        using (RegistryKey d = k.OpenSubKey(sub))
                        {
                            string name = d.GetValue("Name") as string ?? sub;
                            string lib  = d.GetValue("FunctionLibrary") as string;
                            if (lib != null && File.Exists(lib) &&
                                (name.IndexOf("Tactrix", StringComparison.OrdinalIgnoreCase) >= 0 ||
                                 name.IndexOf("OpenPort", StringComparison.OrdinalIgnoreCase) >= 0))
                                return lib;
                        }
                    }
                }
            } catch { }
        }
        return DLL;   // hardcoded fallback
    }

    // ────────────────────────────────────────────────────────────────────────
    static void Main()
    {
        Log("=== SsmInitTiming — SSM pre-flight tool ===");
        Log("");

        // ── Backend selection: Tactrix (J2534) or a generic USB-serial K-line cable ──
        string[] coms = SerialPort.GetPortNames();
        Array.Sort(coms);
        Log("Connect via:");
        Log("   T = Tactrix OpenPort 2.0 (J2534)");
        foreach (string p in coms)
        {
            string num = (p.Length > 3) ? p.Substring(3) : p;   // "COM3" -> "3"
            Log("  " + num + " = " + p + "  (generic USB-serial K-line cable)");
        }
        Console.Write("Press T, or a COM number, then Enter: ");
        string choice = (Console.ReadLine() ?? "").Trim();
        Log("");

        // Handles used at cleanup (assigned in the Tactrix branch only).
        uint devId = 0, chanId = 0;
        dClose Close = null; dDisconnect Disconnect = null;

        if (choice.ToUpperInvariant() == "T")
        {
            // ── Tactrix / J2534 transport ────────────────────────────────────
            g_serial = false;
            string dll = FindJ2534Dll();
            Log("DLL: " + dll);
            IntPtr hMod = LoadLibraryEx(dll, IntPtr.Zero, 0);
            if (hMod == IntPtr.Zero) { Log("LoadLibrary failed: " + Marshal.GetLastWin32Error()); return; }

            var Open        = Fn<dOpen>       (hMod, "PassThruOpen");
            Close           = Fn<dClose>      (hMod, "PassThruClose");
            var ReadVersion = Fn<dReadVersion>(hMod, "PassThruReadVersion");
            var Connect     = Fn<dConnect>    (hMod, "PassThruConnect");
            Disconnect      = Fn<dDisconnect> (hMod, "PassThruDisconnect");
            var Ioctl       = Fn<dIoctl>      (hMod, "PassThruIoctl");
            var StartFilter = Fn<dStartFilter>(hMod, "PassThruStartMsgFilter");
            WriteMsgs    = Fn<dWriteMsgs>  (hMod, "PassThruWriteMsgs");
            ReadMsgs     = Fn<dReadMsgs>   (hMod, "PassThruReadMsgs");
            GetLastError = Fn<dGetLastError>(hMod, "PassThruGetLastError");

            int r = Open(IntPtr.Zero, out devId);
            if (!Check(r, "PassThruOpen")) return;
            var fw = new StringBuilder(256); var dll2 = new StringBuilder(256); var api = new StringBuilder(256);
            ReadVersion(devId, fw, dll2, api);
            Log("Tactrix FW=" + fw + "  DLL=" + dll2);
            Log("");
            r = Connect(devId, ISO9141, ISO9141_NO_CHECKSUM, 4800, out chanId);
            if (!Check(r, "PassThruConnect ISO9141/4800")) { Close(devId); return; }

            // SET_CONFIG one-at-a-time (RomRaider's ISO9141/SSM params).
            uint[]   cfgP = { CFG_PARITY, CFG_DATA_BITS, CFG_P1_MAX, CFG_P3_MIN, CFG_P4_MIN, CFG_LOOPBACK };
            uint[]   cfgV = {  NO_PARITY,             0,          1,          1,          0,            1 };
            string[] cfgN = { "PARITY=0", "DATA_BITS=8", "P1_MAX=1", "P3_MIN=1", "P4_MIN=0", "LOOPBACK=1" };
            for (int k = 0; k < cfgP.Length; k++)
            {
                byte[] c1 = new byte[16];
                PutU32(c1, 0, 1); PutU32(c1, 8, cfgP[k]); PutU32(c1, 12, cfgV[k]);
                GCHandle h1 = GCHandle.Alloc(c1, GCHandleType.Pinned);
                PutU32(c1, 4, (uint)(h1.AddrOfPinnedObject().ToInt64() + 8));
                int rc = Ioctl(chanId, SET_CONFIG, h1.AddrOfPinnedObject(), IntPtr.Zero);
                h1.Free();
                Check(rc, "SET_CONFIG " + cfgN[k]);
            }

            // Pass-all filter.
            byte[] maskMsg = new byte[MSG_SIZE]; byte[] patMsg = new byte[MSG_SIZE];
            PutU32(maskMsg, OFF_PROTO, ISO9141); PutU32(patMsg, OFF_PROTO, ISO9141);
            PutU32(maskMsg, OFF_DSIZE, 1); PutU32(patMsg, OFF_DSIZE, 1);
            GCHandle hMask = GCHandle.Alloc(maskMsg, GCHandleType.Pinned);
            GCHandle hPat  = GCHandle.Alloc(patMsg,  GCHandleType.Pinned);
            uint filtId;
            r = StartFilter(chanId, PASS_FILTER, hMask.AddrOfPinnedObject(),
                        hPat.AddrOfPinnedObject(), IntPtr.Zero, out filtId);
            Check(r, "StartMsgFilter (pass-all)");
            hMask.Free(); hPat.Free();
            Thread.Sleep(100);
        }
        else
        {
            // ── Generic USB-serial K-line transport (CH340 etc.) ─────────────
            g_serial = true;
            int comNum;
            if (!int.TryParse(choice, out comNum))
            { Log("Invalid choice '" + choice + "'. Use T or a COM number."); return; }
            string portName = "COM" + comNum;
            try
            {
                g_port = new SerialPort(portName, 4800, Parity.None, 8, StopBits.One);
                g_port.ReadTimeout = 1000; g_port.WriteTimeout = 1000;
                g_port.Open();
            }
            catch (Exception e) { Log("Failed to open " + portName + ": " + e.Message); return; }
            Log("Opened " + portName + " @ 4800 8N1 (generic K-line cable).");
            Log("NOTE: SSM is 4800-baud-bound, so a generic serial cable performs on par with J2534");
            Log("      (measured: equal throughput, often LOWER jitter than the Tactrix DLL).");
            Log("");
        }

        FindDefs();   // resolve defs: external logger_*.xml if present, else built-in universal subset
        if (g_defsEmbedded)
        {
            Log("Using BUILT-IN universal defs (P-params, switches, DTC codes) — no external file needed.");
            Log("  For ROM-specific extended (E-param) reads, drop the full logger_*.xml next to the .exe.");
            Log("");
        }
        else if (g_defsPath == null)
        {
            Log("WARNING: no definitions available — most decoding (names, snapshot, DTCs) will be SKIPPED.");
            Log("");
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
            Log("No valid init responses from the ECU — aborting.");
            Log("  Check, then re-run:");
            Log("   • Ignition must be ON (key to RUN; engine can be off). Not just accessory.");
            if (g_serial) {
                Log("   • Right COM port? Re-run and pick the number listed for your cable.");
                Log("   • Cable fully seated in the OBD-II port, and it's a K-LINE cable (uses OBD pin 7).");
            } else {
                Log("   • Tactrix cable seated in the OBD-II port and recognised by Windows.");
            }
            Log("   • Car must speak Subaru SSM over K-line (most pre-2015 Subarus do).");
            if (g_serial) { if (g_port != null) g_port.Close(); } else { Disconnect(chanId); Close(devId); }
            return;
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
        TcuProbe(chanId, FindDefs());             // transmission controller (0x18): present? supported params?
        MaxBatchProbe(chanId);                    // largest batch the ECU answers
        BatchOverflowTest(chanId);                // max+1: rejected gracefully, ECU keeps answering?
        BlockReadTest(chanId, FindDefs(), romId);   // 0xA0 block reads: supported? read real memory?
        BlockReadClampTest(chanId);               // confirm the ROM 0xF9/249 A0 max-count clamp on-car
        RemapThresholdProbe(chanId);              // A8 SSM_Get remap cutoff: pins 0x190 (this ROM) vs 0x350 (rimwall)
        GetOffsetSweep(chanId);                   // sweep SSM_Get offsets 0x00-0x18F: which are live vs stub (247 vs 153)
        ErrorProbeTest(chanId);                   // bad checksum / unknown cmd → how does the ECU signal errors?
        if (initResp != null) CapabilityConsistencyCheck(chanId, initResp);   // bitmap stable across inits?
        if (initResp != null) AltInitProbe(chanId, initResp);   // 0x9F alt-init (read-only) vs 0xBF

        // (Single-poll Test 2 removed — firmware only fast-polls; see the continuous sweep below.)
        TimingTest(chanId, FindDefs(), initResp);   // default-set logging rate + full decoded snapshot
        RecoveryTest(chanId);                       // stream stall + re-init — firmware watchdog numbers

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
        Log("  Phase 2 watchdog timeout:   firmware keeps a generous fixed 5000 ms (no single-poll basis)");
        Log("════════════════════════════════════════════════════════════════");

        DtcCheck(chanId, FindDefs(), initResp != null ? initResp[3] : 0);   // list + time DTCs (RomRaider trim by init len)

        AdvancedWriteTests(chanId);   // LAST: mutating write/ROM tests — OPT-IN, engine-OFF, all reversible

        if (g_serial) { if (g_port != null) g_port.Close(); } else { Disconnect(chanId); Close(devId); }
        Log("\nDone.");
    }

    // ── Pre-flight checks ────────────────────────────────────────────────────

    // Build a 3-byte SSM address 0x0000xx (low RAM, where the standard P-params live).
    static byte[] Addr(int low) { return new byte[] { 0x00, 0x00, (byte)low }; }

    // CONTINUOUS-ONLY primitive (mirrors the firmware's fast-poll): start a continuous
    // 0xA8 stream, grab ONE frame, stop + drain. Returns the raw SSM response, or null.
    static byte[] StreamOnce(uint chanId, byte[][] addrs)
    {
        Send(chanId, BuildBatchRead(addrs, true));    // flag=1 → ECU streams
        byte[] r = Recv(chanId, 2500);                // first streamed frame
        Send(chanId, BuildBatchRead(addrs, false));   // any request stops the stream
        // Serial: the next request's SerialSend flushes the input buffer, so no drain needed here.
        // J2534: drain the few in-flight frames so they don't bleed into the next read.
        if (!g_serial)
            for (int k = 0; k < 4; k++) { if (Recv(chanId, 120) == null) break; }
        return r;
    }

    // Read N addresses via a SINGLE one-shot (flag=0) response: the ECU answers once and does
    // NOT stream, so there's no stop request and no drain — ~2x faster than the stream path over
    // the 4800-baud link. Used for one-shot value reads (snapshot, DTC, live-sanity, E-params,
    // switches). The rate/stability TIMING tests deliberately use the continuous path instead.
    static byte[] ReadAddrs(uint chanId, byte[][] addrs)
    {
        Send(chanId, BuildBatchRead(addrs, false));   // flag=0 → single response, no stream
        byte[] r = Recv(chanId, 2500);
        // Response: 80 F0 10 <N+1> E8 <N data bytes> <cs>
        if (r == null || r.Length < 6 + addrs.Length) return null;
        if (r[0] != 0x80 || r[1] != 0xF0 || r[2] != 0x10 || r[4] != 0xE8) return null;
        byte[] data = new byte[addrs.Length];
        Array.Copy(r, 5, data, 0, addrs.Length);
        return data;
    }

    // Block read (0xA0): read `count` CONTIGUOUS bytes from one start address in a single request.
    //   Request:  80 10 F0 06 A0 00 <addr-hi> <addr-mid> <addr-lo> <count-1> <cs>
    //   Response: 80 F0 10 <count+1> E0 <count data bytes> <cs>
    // NOTE: 0xA0 reads RAW memory (unlike low-addr 0xA8 which hits the SSM_Get accessor). Many
    // ECUs/builds don't support it — that's exactly what this test determines.
    static byte[] BuildBlockRead(uint addr, int count)
    {
        byte[] pkt = new byte[11];
        pkt[0] = 0x80; pkt[1] = 0x10; pkt[2] = 0xF0; pkt[3] = 0x06; pkt[4] = 0xA0; pkt[5] = 0x00;
        pkt[6] = (byte)(addr >> 16); pkt[7] = (byte)(addr >> 8); pkt[8] = (byte)addr;
        pkt[9] = (byte)(count - 1);
        pkt[10] = Checksum(pkt, 10);
        return pkt;
    }

    static byte[] BlockRead(uint chanId, uint addr, int count)
    {
        Send(chanId, BuildBlockRead(addr, count));
        byte[] r = Recv(chanId, 2500);
        if (r == null || r.Length < 6 + count) return null;
        if (r[0] != 0x80 || r[1] != 0xF0 || r[2] != 0x10 || r[4] != 0xE0) return null;
        byte[] data = new byte[count];
        Array.Copy(r, 5, data, 0, count);
        return data;
    }

    // First ROM-matched <ecuparam> extended address (and its length) from the defs, or null.
    static string FirstEcuparam(string[] lines, string romId, out int len)
    {
        len = 1;
        bool wantAddr = false;
        foreach (string line in lines)
        {
            if (line.IndexOf("<ecuparam ") >= 0) { wantAddr = false; continue; }
            if (line.IndexOf("<ecu ") >= 0) { string id = Attr(line, "id"); wantAddr = (id != null && romId != null && id.IndexOf(romId) >= 0); continue; }
            if (wantAddr && line.IndexOf("<address") >= 0)
            {
                string lenS = Attr(line, "length"); if (lenS != null) int.TryParse(lenS, out len);
                int gt = line.IndexOf('>', line.IndexOf("<address"));
                int lt = gt >= 0 ? line.IndexOf('<', gt + 1) : -1;
                if (gt < 0 || lt <= gt) { wantAddr = false; continue; }
                return line.Substring(gt + 1, lt - gt - 1).Trim();
            }
        }
        return null;
    }

    // Does this ECU answer 0xA0 block reads, and do they return REAL memory? (Sources disagree.)
    //   Probe 1: a flash/cal address (often 0xFF padding — just shows 0xA0 reaches the region).
    //   Probe 2 (definitive): a KNOWN-POPULATED extended address taken from the first ROM-matched
    //   E-param in the defs — read via 0xA8 (reference) AND 0xA0; if they match and it's non-0xFF,
    //   0xA0 genuinely reads real memory. (At >=0x0350 both 0xA8 and 0xA0 are raw reads.)
    static void BlockReadTest(uint chanId, string defsPath, string romId)
    {
        Log("");
        Log("── 0xA0 block-read support test ─────────────────────────────────────");
        Log("  (does this ECU answer 0xA0 block reads, and do they return real memory?)");

        // Probe 1: flash/cal region.
        uint fa = 0x008000; int fn = 16;
        byte[] fblk = BlockRead(chanId, fa, fn);
        Log(fblk == null
            ? string.Format("    flash  0x{0:X6} ({1}B): NO/BAD RESPONSE", fa, fn)
            : string.Format("    flash  0x{0:X6} ({1}B) via 0xA0: {2}", fa, fn, Hex(fblk)));

        // Probe 2: known-populated E-param extended address (real, non-0xFF data).
        if (defsPath == null || !File.Exists(defsPath) || romId == null)
        {
            Log("    (no external defs/ROM match — run with the full logger_*.xml for the real-data check.)");
            return;
        }
        int len; string addrHex = FirstEcuparam(File.ReadAllLines(defsPath), romId, out len);
        if (addrHex == null) { Log("    (no ROM-matched E-param address available to cross-check 0xA0.)"); return; }
        uint a;
        try { a = HexU(addrHex); } catch { Log("    (couldn't parse E-param address " + addrHex + ")"); return; }
        byte[] r8 = ReadAddrs(chanId, ConsecAddrs(ParseAddr(addrHex), len));
        byte[] r0 = BlockRead(chanId, a, len);
        if (r0 == null)
        {
            Log(string.Format("    E-param 0x{0:X6} ({1}B): 0xA0 NO/BAD RESPONSE  (0xA8={2})", a, len, r8 == null ? "?" : Hex(r8)));
            Log("    -> 0xA0 does NOT read this extended region (may be a protected/register carve-out).");
            return;
        }
        bool match = r8 != null;
        if (r8 != null) for (int i = 0; i < len; i++) if (r0[i] != r8[i]) { match = false; break; }
        Log(string.Format("    E-param 0x{0:X6} ({1}B): 0xA0={2}  0xA8={3}", a, len, Hex(r0), r8 == null ? "?" : Hex(r8)));
        Log(match
            ? "    -> MATCH on real (non-FF) data → 0xA0 genuinely reads memory correctly on this ECU."
            : "    -> differ → either a live value changed between reads, or 0xA0 reads a different region.");
    }

    // Send a batch ONE larger than the probed ceiling. The firmware must never exceed the ECU's
    // max, but if it ever did we want proof the ECU rejects the over-sized request cleanly and
    // keeps answering (rather than wedging the session). Read-only — fully safe.
    static void BatchOverflowTest(uint chanId)
    {
        Log("");
        Log("── Batch-overflow graceful-failure test ─────────────────────────────");
        // Send a batch CLEARLY over the probed ceiling (max+4, not max+1 — the probe steps in
        // 2-4 increments near the top, so max+1 may still be within the real limit). Use a SINGLE
        // (flag=0) read so there's no continuous stream to clean up afterward.
        int n = (g_maxBatch > 0 ? g_maxBatch : 40) + 4;
        var addrs = new byte[n][];
        for (int i = 0; i < n; i++) addrs[i] = new byte[] { 0x00, 0x00, (byte)i };
        Send(chanId, BuildBatchRead(addrs, false));
        byte[] resp = Recv(chanId, 2000);
        bool answered = resp != null && resp.Length >= 6 + n && resp[4] == 0xE8;
        Log(string.Format("    {0} addr (probed max {1} + 4) -> {2}", n, g_maxBatch,
            answered ? "unexpectedly ANSWERED (" + resp.Length + " bytes)" : "rejected / no response (expected)"));
        // Drain anything in-flight, then confirm the ECU still answers — retry to absorb a transient
        // framing hiccup so a single dropped read isn't misreported as a wedged session.
        for (int k = 0; k < 6; k++) { if (Recv(chanId, 120) == null) break; }
        byte[] d = null;
        for (int t = 0; t < 3 && d == null; t++) d = ReadAddrs(chanId, new byte[][] { Addr(0x08) });
        Log("    normal read after overflow: " + (d != null ? "OK (ECU fine)" : "FAILED (3 tries)"));
        Log(d != null
            ? "    -> over-sized batch handled cleanly (rejected, ECU keeps answering). Firmware's probed ceiling is safe."
            : "    -> ECU didn't answer a normal read in 3 tries afterward — investigate (later tests will show if truly wedged).");
    }

    // Build a single-byte address write (0xB8): 80 10 F0 05 B8 <ah> <am> <al> <data> <cs>
    static byte[] BuildAddrWrite(byte[] addr, byte data)
    {
        byte[] pkt = new byte[10];
        pkt[0] = 0x80; pkt[1] = 0x10; pkt[2] = 0xF0; pkt[3] = 0x05; pkt[4] = 0xB8;
        pkt[5] = addr[0]; pkt[6] = addr[1]; pkt[7] = addr[2]; pkt[8] = data;
        pkt[9] = Checksum(pkt, 9);
        return pkt;
    }

    // Single-byte write via 0xB8. Response: 80 F0 10 02 F8 <data> <cs> — echoes the written byte.
    // Returns the echoed byte, or -1 on no/bad response.
    static int AddrWrite(uint chanId, byte[] addr, byte data)
    {
        Send(chanId, BuildAddrWrite(addr, data));
        byte[] r = Recv(chanId, 2500);
        if (r == null || r.Length < 7) return -1;
        if (r[0] != 0x80 || r[1] != 0xF0 || r[2] != 0x10 || r[4] != 0xF8) return -1;
        return r[5];
    }

    // Build a block write (0xB0): 80 10 F0 <len> B0 <ah><am><al> <data...> <cs>  (NO pad byte,
    // unlike 0xA0/0xA8 reads which carry a PP single/continuous flag). len = 4 + N data bytes.
    // True if a block-read result is all-0xFF or null (the ECU's "blocked / unmapped" signature).
    static bool IsBlocked(byte[] d)
    {
        if (d == null) return true;
        foreach (byte b in d) if (b != 0xFF) return false;
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ADVANCED WRITE / ROM TESTS — mutating, engine-OFF only, run LAST.
    // Exercises the SSM write paths (0xB8 SSM_Set, 0xB0 block write) + the
    // ROM-read "special byte" unlock + DTC-clear-without-reset. EVERY mutating
    // step restores the original state and verifies it. Runs automatically once
    // the engine-running guard passes (RPM <= 50); no YES prompt.
    // ═══════════════════════════════════════════════════════════════════════
    static void AdvancedWriteTests(uint chanId)
    {
        Log("");
        Log("════════════════════════════════════════════════════════════════");
        Log("ROM-READ UNLOCK + REGION MAP  (one reversible write, engine-OFF)");
        Log("════════════════════════════════════════════════════════════════");

        RegionProbe(chanId);   // read-only — always safe

        // Engine-off guard: the unlock writes one gate bit; only do it engine-off (offset 0 is
        // unverified on non-A2WC521R ECUs, so stay conservative even though it's ROM-read access).
        byte[] rpmD = ReadAddrs(chanId, new byte[][] { Addr(0x0E), Addr(0x0F) });
        double rpm = (rpmD != null) ? (rpmD[0] * 256 + rpmD[1]) / 4.0 : -1;
        if (rpm > 50)
        {
            Log("");
            Log(string.Format("  ENGINE IS RUNNING (~{0:F0} rpm) — ROM-unlock write SKIPPED for safety.", rpm));
            return;
        }

        // The ROM-read unlock is the ONLY remaining write: clears one gate bit, restores the EXACT
        // original → reversible on ANY ECU. (The P239 round-trip + 0xB0 no-op validation tests were
        // retired 2026-06-13 — fully confirmed on A2WC521R and settled by the ROM disassembly.)
        SpecialByteUnlockTest(chanId);
    }

    // Read-only: 0xA0-sample one address per memory region and report readable vs blocked.
    // (3-byte SSM addresses; top byte 0xFF maps to the ECU's physical 0xFFFFxx RAM/registers.)
    static void RegionProbe(uint chanId)
    {
        Log("");
        Log("── Memory-region permission probe (0xA0, read-only) ─────────────────");
        Log("  (probes the ROM-derived permission boundaries; flash reads blocked until special byte)");
        uint[] addrs   = { 0x000100, 0x002000, 0x0C0000, 0x0C0034, 0xFF2000, 0xFF2C64, 0xFFBFE0 };
        string[] labels = {
            "bootloader/IO          (perm-map: no access)",
            "ECU-logic start 0x2000 (read, special-byte gated)",
            "cal block 0xC0000      (BELOW maps boundary — special-byte gated)",
            "maps proper 0xC0034    (NOT gated — reads at default lock)",
            "RAM 0xFFFF2000         (perm-map: read)",
            "RAM R/W edge 0xFFFF2C64 (rimwall boundary)",
            "RAM 0xFFFFBFE0         (perm-map R/W vs wiki carve-out >0x3FE4 — resolves which)" };
        byte[][] res = new byte[addrs.Length][];
        for (int i = 0; i < addrs.Length; i++)
        {
            res[i] = BlockRead(chanId, addrs[i], 4);
            Log(string.Format("    0x{0:X6}  {1,-54} {2}", addrs[i], labels[i],
                res[i] == null ? "blocked / no response" : (IsBlocked(res[i]) ? "all-FF (blocked/unmapped)" : "READ " + Hex(res[i]))));
        }
        // Maps-gate boundary demo (ROM-derived, on-car proven): at default lock the special byte gates
        // [0x2000,0xC0034), so 0xC0000 is blocked but 0xC0034 (maps proper) reads — pins the boundary live.
        if (IsBlocked(res[2]) && !IsBlocked(res[3]))
            Log("    -> maps-gate boundary CONFIRMED at 0xC0034: 0xC0000 blocked, 0xC0034 reads (special byte still locked).");
    }

    // Item 1 — prove a real 0xB8 write via an observable side-effect that touches NOTHING engine
    // related: clear the ROM-read "special byte" (SSM_Set offset 0, default 0x04), watch the
    // ECU-logic region 0x008000 go from blocked→readable, then restore 0x04 and confirm re-locked.
    static void SpecialByteUnlockTest(uint chanId)
    {
        Log("");
        Log("── ROM-read unlock (special byte @ SSM_Set offset 0) ────────────────");
        Log("  (clears the ROM-read gate bit, checks a blocked region became readable, restores EXACTLY)");
        byte[] sbAddr = Addr(0x00);
        uint region = 0x008000; int n = 16;
        byte[] sbR = ReadAddrs(chanId, new byte[][] { sbAddr });
        if (sbR == null) { Log("    couldn't read offset 0 — skipping unlock."); return; }
        byte sb = sbR[0];
        byte unlk = (byte)(sb & ~0x04);                  // clear ONLY bit-2; preserve every other bit (cross-ECU safe)
        Log(string.Format("    offset 0 (gate byte) = 0x{0:X2}  (A2WC521R default is 0x04)", sb));

        byte[] before = BlockRead(chanId, region, n);
        Log("    ROM 0x008000 before: " + (IsBlocked(before) ? "blocked (all-FF/none)" : "READ " + Hex(before)));

        int echo = AddrWrite(chanId, sbAddr, unlk);      // attempt unlock (bit-2 cleared)
        Log(echo < 0 ? "    0xB8 write: NO/BAD RESPONSE — offset 0 not writable on this ECU."
                     : string.Format("    wrote 0x{0:X2} (bit-2 cleared) -> 0xF8 echo 0x{1:X2}", unlk, echo));
        byte[] after = BlockRead(chanId, region, n);
        Log("    ROM 0x008000 after:  " + (IsBlocked(after) ? "still blocked" : "READ " + Hex(after)));

        // ALWAYS restore the EXACT original value (reversible on any ECU), then verify.
        int rEcho = AddrWrite(chanId, sbAddr, sb);
        byte[] back = ReadAddrs(chanId, new byte[][] { sbAddr });
        bool restored = (back != null && back[0] == sb);
        byte[] relock = BlockRead(chanId, region, n);
        Log(string.Format("    restored 0x{0:X2} (echo 0x{1:X2}); offset-0 read-back 0x{2:X2} {3}; region {4}",
            sb, rEcho, back != null ? back[0] : 0, restored ? "(ok)" : "(RESTORE FAILED!)",
            IsBlocked(relock) ? "blocked again" : "still readable"));
        if (!restored)
        {
            Log("    ****************************************************************");
            Log(string.Format("    ** RESTORE FAILED — offset 0 may be left modified. Original was 0x{0:X2}.", sb));
            Log(string.Format("    ** Manually set it back: RR Test App -> Write, Addr 0x000000, Data 0x{0:X2}.", sb));
            Log("    ****************************************************************");
        }

        bool didUnlock = IsBlocked(before) && !IsBlocked(after);
        Log(didUnlock
            ? "    -> UNLOCK WORKS on this ECU: clearing bit-2 revealed ROM; exact-restore re-locked it."
            : (echo >= 0 ? "    -> write accepted but region didn't change (already open, offset-0 isn't the gate here, or read otherwise blocked)."
                         : "    -> write not accepted — offset 0 isn't writable on this ECU."));
    }

    // (P239 round-trip + 0xB0 no-op validation tests removed 2026-06-13 — fully confirmed on A2WC521R
    //  + settled by ROM disassembly; only the ROM-read unlock above has ongoing utility. DTC-clear test — the SSM_Set 0x60 clear has an
    //  ambiguous side effect on readiness monitors / adaptive memory; dropped at user request.)

    // What does the ECU do with a corrupted or unknown request? This decides the firmware's
    // error handling: if the ECU answers bad frames with NOTHING (no NAK), then a receive
    // TIMEOUT is the firmware's ONLY error signal and must be treated as such.
    static void ErrorProbeTest(uint chanId)
    {
        Log("");
        Log("── Error-behavior probes (bad checksum / unknown command) ───────────");

        // Probe A: a valid 1-addr read with the checksum deliberately corrupted (+1).
        byte[] pkt = BuildBatchRead(new byte[][] { Addr(0x08) }, false);
        pkt[pkt.Length - 1]++;
        Send(chanId, pkt);
        byte[] r = Recv(chanId, 1000);
        Log("    bad checksum    -> " + (r == null ? "SILENCE (no response within 1 s)" : "response: " + Hex(r)));

        // Probe B: an unknown command byte (0x99 is not in the SSM2 command set).
        byte[] u = new byte[6];
        u[0] = 0x80; u[1] = 0x10; u[2] = 0xF0; u[3] = 0x01; u[4] = 0x99; u[5] = Checksum(u, 5);
        Send(chanId, u);
        r = Recv(chanId, 1000);
        Log("    unknown cmd 99  -> " + (r == null ? "SILENCE (no response within 1 s)" : "response: " + Hex(r)));

        // Recovery: a normal read must still work immediately after the bad frames.
        byte[] d = ReadAddrs(chanId, new byte[][] { Addr(0x08) });
        Log("    normal read after probes: " + (d != null ? "OK (ECU unaffected)" : "FAILED"));
        Log(d != null
            ? "    -> ECU ignores bad frames without breaking the session; firmware: timeout = the only error signal."
            : "    -> WARNING: ECU stopped answering after bad frames — firmware needs a full re-init here.");
    }

    // Send 0xBF twice more and byte-compare against the first init response. If the
    // capability bitmap is stable, the firmware can read it once at startup and cache it.
    static void CapabilityConsistencyCheck(uint chanId, byte[] firstResp)
    {
        Log("");
        Log("── Capability-response consistency (3 inits, byte-compare) ──────────");
        byte[] initPkt = BuildInit();
        bool allSame = true; int got = 0;
        for (int i = 0; i < 2; i++)
        {
            Thread.Sleep(200);
            Send(chanId, initPkt);
            byte[] r = Recv(chanId, 3000);
            if (r == null) { Log("    [" + (i + 2) + "] TIMEOUT"); allSame = false; continue; }
            got++;
            bool same = (r.Length == firstResp.Length);
            if (same) for (int k = 0; k < r.Length; k++) if (r[k] != firstResp[k]) { same = false; break; }
            Log("    [" + (i + 2) + "] " + r.Length + " bytes — " + (same ? "identical to first init" : "DIFFERS from first init!"));
            if (!same) { allSame = false; Log("        " + Hex(r)); }
        }
        Log(allSame && got == 2
            ? "    -> init response (ROM ID + capability bitmap) is stable — firmware can read it once and cache."
            : "    -> WARNING: init responses varied across reads — firmware should not blindly cache the bitmap.");
    }

    // Simulate the firmware's worst case: a continuous stream is running, the logger stalls
    // (SD write hang, brownout, wiring blip) and stops reading, then must recover. Measures
    // how long a 0xBF re-init takes from inside a live stream — the firmware's watchdog recipe.
    static void RecoveryTest(uint chanId)
    {
        Log("");
        Log("── Stream-interruption recovery test ────────────────────────────────");
        Log("  (start stream, stall 1.5 s, re-init — measures the firmware's recovery path)");

        byte[][] addrs = new byte[][] { Addr(0x0E), Addr(0x0F), Addr(0x08), Addr(0x10), Addr(0x15), Addr(0x1C) };
        Send(chanId, BuildBatchRead(addrs, true));
        int pre = 0;
        for (int i = 0; i < 3; i++) { byte[] f = Recv(chanId, 1500); if (f == null) break; pre++; }
        if (pre == 0)
        {
            Log("    stream did not start — skipping.");
            Send(chanId, BuildBatchRead(addrs, false));
            for (int k = 0; k < 6; k++) { if (Recv(chanId, 150) == null) break; }
            return;
        }
        Log("    streaming OK (" + pre + " frames) — stalling 1.5 s (ECU keeps streaming)...");
        Thread.Sleep(1500);

        // Recovery: send 0xBF (any request stops the stream), then resync past any stale
        // buffered 0xE8 stream frames until the init response (cmd 0xFF) arrives.
        long t0 = Stopwatch.GetTimestamp();
        byte[] initPkt = BuildInit();
        int attempts = 0, stale = 0; bool ok = false;
        while (!ok && attempts < 5)
        {
            attempts++;
            Send(chanId, initPkt);
            for (int k = 0; k < 12; k++)             // wade through buffered stream frames
            {
                byte[] r = Recv(chanId, 800);
                if (r == null) break;
                if (r.Length >= 6 && r[4] == 0xFF) { ok = true; break; }
                stale++;
            }
        }
        double recMs = Ticks2Ms(Stopwatch.GetTimestamp() - t0);
        if (!ok) { Log("    re-init FAILED after " + attempts + " attempts — ECU may need a quiet period first."); return; }
        Log(string.Format("    re-init OK: attempt {0}, {1} stale stream frame(s) skipped, {2:F0} ms total.", attempts, stale, recMs));

        // Confirm a fresh stream starts cleanly after the recovery.
        byte[] f2 = StreamOnce(chanId, addrs);
        bool restart = (f2 != null && f2.Length >= 6 && f2[4] == 0xE8);
        Log("    fresh stream after recovery: " + (restart ? "OK" : "FAILED"));
        Log(string.Format("    -> firmware recovery recipe: send 0xBF, skip stale 0xE8 frames, expect init within ~{0:F0} ms.", recMs));
    }

    // ROM RE finding: the A0 block-read count is clamped to 0xF9 (249) by the decoder. Confirm on-car:
    // request 249 then 250 bytes from a readable RAM region and report the ACTUAL data length returned.
    // Read-only (RAM @ 0xFFFF2000 isn't special-byte gated). Expect both to return 249 → clamp confirmed.
    static void BlockReadClampTest(uint chanId)
    {
        Log("");
        Log("── 0xA0 max-count clamp (ROM says 249 / 0xF9) ───────────────────────");
        uint addr = 0xFF2000;                        // physical 0xFFFF2000 — readable RAM
        foreach (int req in new int[] { 249, 250 })
        {
            Send(chanId, BuildBlockRead(addr, req));
            byte[] r = Recv(chanId, 3000);
            if (r == null || r.Length < 6 || r[4] != 0xE0)
            { Log(string.Format("    requested {0,3} -> no/bad response", req)); continue; }
            int dataLen = r[3] - 1;                   // resp len byte = E0 + data → data = len-1
            Log(string.Format("    requested {0,3} -> returned {1,3} bytes{2}", req, dataLen,
                (req > 249 && dataLen == 249) ? "  (CLAMPED at 249)" : (dataLen == req ? "  (exact)" : "")));
        }
        Log("    -> a >249 request returning exactly 249 confirms the ROM 0xF9 block-read clamp.");
    }

    // A8 SSM_Get remap DEMONSTRATION (read-only). For 0xA8 with top byte != 0xFF, addresses below a
    // cutoff are remapped to the SSM_Get accessor (a live computed param) instead of raw memory. We prove
    // the remap is real by reading the SAME low address two ways: A8 (goes through SSM_Get) vs A0 (raw
    // block read). Below the cutoff they DIFFER (A8 = live param, A0 = raw byte — and since 0x0–0x2000 is
    // the blocked bootloader region, the raw byte is 0xFF). For a high addr (top byte 0xFF) both are raw
    // reads → they MATCH. NOTE: the exact cutoff is ROM-derived (0x190 = the 400-entry Get-table size);
    // 0x190 vs rimwall's 0x350 is NOT distinguishable on-car because [0x190,0x2000) reads 0xFF either way.
    static void RemapThresholdProbe(uint chanId)
    {
        Log("");
        Log("── A8 SSM_Get remap demonstration (A8 vs A0 at the same address) ─────");
        Log("  (A8 of a low addr = SSM_Get accessor (live param); A0 = raw memory. DIFFER below the cutoff.)");
        int[] lowOff = { 0x08, 0x0D, 0x1C };
        string[] lowName = { "coolant", "MAP", "battery" };
        for (int i = 0; i < lowOff.Length; i++)
        {
            byte[] a8 = ReadAddrs(chanId, new byte[][] { new byte[] { 0, 0, (byte)lowOff[i] } });
            byte[] a0 = BlockRead(chanId, (uint)lowOff[i], 1);
            string s8 = a8 != null ? string.Format("0x{0:X2}", a8[0]) : "no resp";
            string s0 = a0 != null ? string.Format("0x{0:X2}", a0[0]) : "no resp";
            string v = (a8 != null && a0 != null)
                ? (a8[0] != a0[0] ? "DIFFER -> A8 remapped to SSM_Get (live param)" : "match (unexpected for a low addr)")
                : "";
            Log(string.Format("    0x{0:X6} ({1,-8}): A8={2,-7} A0={3,-7} {4}", lowOff[i], lowName[i], s8, s0, v));
        }
        // High address (top byte 0xFF): both A8 and A0 are raw reads -> should MATCH (no remap).
        byte[] h8 = ReadAddrs(chanId, new byte[][] { new byte[] { 0xFF, 0x26, 0x7C } });
        byte[] h0 = BlockRead(chanId, 0xFF267C, 1);
        if (h8 != null && h0 != null)
            Log(string.Format("    0xFF267C (E-param): A8=0x{0:X2}  A0=0x{1:X2}  {2}", h8[0], h0[0],
                h8[0] == h0[0] ? "MATCH -> raw read, no remap (top byte 0xFF / above cutoff)" : "differ (unexpected)"));
        Log("    -> low-addr A8 != A0 proves the SSM_Get remap is live (A8 = computed param, A0 = raw byte).");
        Log("       Exact cutoff is ROM-derived: 0x190 (= 400-entry Get-table size). 0x190 vs rimwall's 0x350");
        Log("       is NOT on-car-distinguishable — [0x190,0x2000) reads 0xFF whether remapped or raw.");
    }

    // SSM_Get readable-offset sweep (read-only). All A8 reads with addr < the remap cutoff (0x190) go
    // through the SSM_Get accessor table. The ROM dispatch RE found ~247 of the 400 offsets are real
    // getters and 153 are stubs that return 0x00. This sweeps 0x000000–0x00018F and prints a value grid:
    // non-zero = a definitely-live readable offset; '..' = 0x00 (stub OR an implemented param currently 0).
    // Surfaces unadvertised-but-readable engineering values (live here yet not capability-flagged).
    static void GetOffsetSweep(uint chanId)
    {
        Log("");
        Log("── SSM_Get readable-offset sweep (A8 0x000000–0x00018F) ──────────────");
        Log("  (ROM: ~247/400 offsets implemented, 153 stubs return 0x00. non-zero = live; '..' = 0x00.)");
        byte[] all = new byte[0x190];
        bool anyFail = false;
        for (int baseOff = 0; baseOff < 0x190; baseOff += 40)
        {
            int cnt = Math.Min(40, 0x190 - baseOff);
            var addrs = new byte[cnt][];
            for (int i = 0; i < cnt; i++)
            { int a = baseOff + i; addrs[i] = new byte[] { (byte)(a >> 16), (byte)(a >> 8), (byte)a }; }
            byte[] d = ReadAddrs(chanId, addrs);
            if (d == null) { anyFail = true; continue; }   // leave that block as 0x00
            Array.Copy(d, 0, all, baseOff, cnt);
        }
        int nz = 0;
        for (int row = 0; row < 0x190; row += 16)
        {
            var sb = new StringBuilder();
            sb.AppendFormat("    0x{0:X4}: ", row);
            for (int i = 0; i < 16 && row + i < 0x190; i++)
            {
                byte b = all[row + i];
                if (b != 0) nz++;
                sb.Append(b == 0 ? ".. " : string.Format("{0:X2} ", b));
            }
            Log(sb.ToString());
        }
        Log(string.Format("    -> {0} of 400 offsets returned non-zero (live readable){1}.",
            nz, anyFail ? "  [some batches failed]" : ""));
        Log("       Offsets live here but NOT among the ~111 capability-flagged params are the ECU's");
        Log("       unadvertised engineering values (diag/state bytes standard tools never request).");
    }

    // 0x9F alt-init — read-only ID query, same class as 0xBF (NO writes, no engine effect). Per the ROM
    // RE it returns 0xDF + SSM-ID + ROM-ID sourced from RAM (0xFFFF64B4-B8) + the 48-byte capability
    // block. Diff it against the 0xBF response: the capability block should be byte-identical, and the
    // RAM-sourced ID should match the cal-ROM ID (or we flag the difference).
    static void AltInitProbe(uint chanId, byte[] bfResp)
    {
        Log("");
        Log("── 0x9F alt-init probe (read-only; compared vs 0xBF) ────────────────");
        byte[] pkt = new byte[6];
        pkt[0] = 0x80; pkt[1] = 0x10; pkt[2] = 0xF0; pkt[3] = 0x01; pkt[4] = 0x9F; pkt[5] = Checksum(pkt, 5);
        Send(chanId, pkt);
        byte[] r = Recv(chanId, 3000);
        if (r == null) { Log("    no response — 0x9F not answered by this ECU."); return; }
        if (r.Length < 13 || r[4] != 0xDF)
        {
            Log("    unexpected response (cmd " + (r.Length > 4 ? "0x" + r[4].ToString("X2") : "??") + "): " + Hex(r));
            return;
        }
        Log("    0x9F -> " + r.Length + " bytes, resp cmd 0xDF (= 0x9F|0x40)  OK");
        Log("    SSM-ID " + Hex2(r, 5, 3) + "   ROM-ID " + Hex2(r, 8, 5) + "  (sourced from RAM per ROM RE)");
        if (bfResp != null && bfResp.Length == r.Length)
        {
            bool idSame = true;
            for (int i = 5; i <= 12 && i < r.Length; i++) if (r[i] != bfResp[i]) { idSame = false; break; }
            bool capSame = true; int end = r.Length - 1;
            for (int i = 13; i < end; i++) if (r[i] != bfResp[i]) { capSame = false; break; }
            Log("    vs 0xBF: ID " + (idSame ? "identical" : "DIFFERS") + ", capability block " + (capSame ? "identical" : "DIFFERS"));
            Log(idSame && capSame
                ? "    -> 0x9F returns the same ID + capabilities as 0xBF (RAM-sourced ID matches cal-ROM)."
                : "    -> 0x9F differs from 0xBF (see above) — the RAM ID/cap block is not identical to cal-ROM.");
        }
        else Log("    (no comparable 0xBF response — skipping diff)");
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
        MapCapabilitiesFor(r, defsPath, false, "Supported P-parameters (ECU)");
    }

    // Map a capability bitmap → supported param NAMES, filtered to ONE module.
    // isTcu=false → ECU params (target != "2"); isTcu=true → TCU params (target == "2").
    // (Filtering by target fixes a latent bug: TCU params were being indexed against the ECU bitmap.)
    static void MapCapabilitiesFor(byte[] r, string defsPath, bool isTcu, string header)
    {
        Log("");
        Log("── " + header + " (capability bit -> name via defs) ──");
        if (defsPath == null || !File.Exists(defsPath))
        {
            Log("    (no logger_*.xml — skipping name mapping)");
            return;
        }
        int supported = 0, gated = 0;
        foreach (string line in File.ReadAllLines(defsPath))
        {
            if (line.IndexOf("</protocol>") >= 0) break;   // SSM is the first protocol; stop before OBD
            if (line.IndexOf("<parameter ") < 0) continue;
            bool tcuParam = (Attr(line, "target") == "2");
            if (tcuParam != isTcu) continue;               // only this module's params
            string bi = Attr(line, "ecubyteindex");
            string bt = Attr(line, "ecubit");
            if (bi == null || bt == null) continue;
            int byteIdx, bit;
            if (!int.TryParse(bi, out byteIdx) || !int.TryParse(bt, out bit)) continue;
            gated++;
            int idx = 5 + byteIdx;     // firmware: rawResp[5 + byteIndex]
            if (idx < r.Length && (r[idx] & (1 << bit)) != 0)
            {
                supported++;
                Log(string.Format("    {0,-5} {1}", Attr(line, "id"), Attr(line, "name")));
            }
        }
        Log("    -> " + supported + " of " + gated + " capability-gated params supported.");
    }

    // TCU (transmission controller) probe — read-only. Inits the TCU at dst 0x18 (same 0xBF init,
    // different destination byte), decodes its ID + capability bitmap, and lists supported TCU
    // params by name (the target="2" defs). A manual-trans car / car with no SSM TCU just times out.
    static void TcuProbe(uint chanId, string defsPath)
    {
        Log("");
        Log("════════════════════════════════════════════════════════════════");
        Log("TCU (transmission) probe — init 0x18, supported parameters");
        Log("════════════════════════════════════════════════════════════════");
        Send(chanId, BuildInitDst(0x18));
        byte[] r = Recv(chanId, 3000);
        if (r == null)
        {
            Log("  No response from the TCU (0x18) — manual transmission, or no SSM TCU on this car.");
            return;
        }
        if (r.Length < 13 || r[4] != 0xFF)
        {
            Log("  Unexpected TCU init response: " + Hex(r));
            return;
        }
        var id = new StringBuilder();
        for (int b = 8; b <= 12; b++) id.AppendFormat("{0:X2}", r[b]);
        int capLen = (r.Length - 1) - 13;   // bytes after the 5-byte ID, before checksum
        Log(string.Format("  TCU present: {0}-byte init, ID {1}, {2} capability bytes.", r.Length, id, capLen < 0 ? 0 : capLen));
        MapCapabilitiesFor(r, defsPath, true, "Supported TCU parameters");
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
        if (d != null) Log(string.Format("    Coolant temp:    {0,7:F0} F      (raw {1:X2})", 32 + 9 * (d[0] - 40.0) / 5, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x12) });
        if (d != null) Log(string.Format("    Intake air temp: {0,7:F0} F      (raw {1:X2})", 32 + 9 * (d[0] - 40.0) / 5, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x15) });
        if (d != null) Log(string.Format("    Throttle:        {0,7:F1} %      (raw {1:X2})", d[0] * 100.0 / 255.0, d[0]));
        d = ReadAddrs(chanId, new[] { Addr(0x10) });
        if (d != null) Log(string.Format("    Vehicle speed:   {0,7:F0} mph    (raw {1:X2})", d[0] * 0.621371192, d[0]));
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
            byte[] resp = StreamOnce(chanId, addrs);   // probe with the continuous path
            bool ok = resp != null && resp.Length >= 6 + n && resp[4] == 0xE8;
            Log(string.Format("    {0,3} addr -> {1}", n, ok ? "OK (" + resp.Length + " bytes)" : "no/bad response"));
            if (ok) lastOk = n; else break;
            Thread.Sleep(30);
        }
        Log("    -> ECU answered batches up to " + lastOk + " addresses.");
        if (lastOk > 0) g_maxBatch = lastOk;   // reuse the real ceiling for all later chunked reads
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
    static string g_defsPath = null;     // cached resolved defs path
    static bool   g_defsResolved = false;
    static bool   g_defsEmbedded = false;  // true when falling back to the built-in universal subset

    // Resolve the defs: an external logger_*.xml next to the .exe (full, ROM-specific E-params work)
    // takes priority; otherwise fall back to the BUILT-IN universal subset (P-params/switches/DTCs)
    // embedded in the .exe, so the tool works standalone like it would on an undefined ECU.
    static string FindDefs()
    {
        if (g_defsResolved) return g_defsPath;
        g_defsResolved = true;
        try {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            string primary = Path.Combine(dir, "logger_IMP_EN_v370.xml");
            if (File.Exists(primary)) { g_defsPath = primary; return g_defsPath; }
            string[] files = Directory.GetFiles(dir, "logger_*.xml");
            foreach (string f in files)
                if (f.IndexOf("_EN_", StringComparison.OrdinalIgnoreCase) >= 0 &&
                    f.IndexOf("universal", StringComparison.OrdinalIgnoreCase) < 0)
                { g_defsPath = f; return g_defsPath; }
            foreach (string f in files)
                if (f.IndexOf("universal", StringComparison.OrdinalIgnoreCase) < 0)
                { g_defsPath = f; return g_defsPath; }
        } catch { }
        // No external defs — extract the embedded universal subset to a temp file.
        g_defsPath = ExtractEmbeddedDefs();
        g_defsEmbedded = (g_defsPath != null);
        return g_defsPath;
    }

    // Decompress the embedded gzipped universal defs to a temp file; return its path (or null).
    static string ExtractEmbeddedDefs()
    {
        try {
            System.Reflection.Assembly asm = System.Reflection.Assembly.GetExecutingAssembly();
            string resName = null;
            foreach (string n in asm.GetManifestResourceNames())
                if (n.IndexOf("universal", StringComparison.OrdinalIgnoreCase) >= 0) { resName = n; break; }
            if (resName == null) return null;
            string tmp = Path.Combine(Path.GetTempPath(), "sssal_universal_defs.xml");
            using (Stream rs = asm.GetManifestResourceStream(resName))
            using (System.IO.Compression.GZipStream gz = new System.IO.Compression.GZipStream(rs, System.IO.Compression.CompressionMode.Decompress))
            using (FileStream fs = new FileStream(tmp, FileMode.Create, FileAccess.Write))
            {
                byte[] buf = new byte[8192]; int got;
                while ((got = gz.Read(buf, 0, buf.Length)) > 0) fs.Write(buf, 0, got);
            }
            return tmp;
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
        if (tested == 0) Log("    UNKNOWN ECU (ROM " + romId + " not in defs) — E-param test skipped (E-params are ROM-specific).");
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

    // Real logging rate + a full decoded snapshot — ECU-agnostic (addresses + formulas come
    // from the defs + the ECU's own capability response; nothing hardwired to a specific car).
    //   (a) default-set: fast-poll the 12 default-profile params → sustained logging Hz.
    //   (b) snapshot: read EVERY capability-supported P-param once, decode to engineering
    //       units via the universal <conversion> formulas, and time the full read.
    static void TimingTest(uint chanId, string defsPath, byte[] initResp)
    {
        Log("");
        Log("── Logging update rate vs payload (continuous fast-poll) ─────────────");
        if (defsPath == null || !File.Exists(defsPath)) { Log("    (no defs — skipping)"); return; }
        string[] lines = File.ReadAllLines(defsPath);

        // default 12-param profile address list (skips any the ECU doesn't support)
        string[] basic = { "P8","P2","P9","P7","P11","P10","P12","P13","P3","P4","P14","P17" };
        var dAddrs = new List<byte[]>();
        foreach (string id in basic)
        {
            PInfo pi = LookupParam(lines, id);
            if (pi == null || pi.addrHex == null) continue;
            bool isSup = !pi.hasCap || (initResp != null && (5 + pi.byteIdx) < initResp.Length && (initResp[5 + pi.byteIdx] & (1 << pi.bit)) != 0);
            if (!isSup) continue;
            foreach (byte[] a in ConsecAddrs(ParseAddr(pi.addrHex), pi.len)) dAddrs.Add(a);
        }

        // Enumerate supported P-params, then GROUP by (address,len) so generation-variant
        // duplicates (same bit + same address, different formula — e.g. P65/P89) collapse.
        List<SupParam> sup = EnumSupportedParams(lines, initResp);
        var keys  = new List<string>();
        var byKey = new Dictionary<string, List<SupParam>>();
        foreach (SupParam p in sup)
        {
            string k = p.addrHex + ":" + p.len;
            List<SupParam> g;
            if (!byKey.TryGetValue(k, out g)) { g = new List<SupParam>(); byKey[k] = g; keys.Add(k); }
            g.Add(p);
        }
        // flat UNIQUE-address read list + per-group offset
        var flat = new List<byte[]>();
        int[] off = new int[keys.Count];
        for (int i = 0; i < keys.Count; i++)
        {
            off[i] = flat.Count;
            SupParam p0 = byKey[keys[i]][0];
            foreach (byte[] a in ConsecAddrs(ParseAddr(p0.addrHex), p0.len)) flat.Add(a);
        }

        // (1) rate vs payload: small / default / full-batch sustained Hz
        if (dAddrs.Count > 0)
        {
            byte[][] half = dAddrs.GetRange(0, Math.Max(1, dAddrs.Count / 2)).ToArray();
            MeasureContinuous(chanId, half, "half-default (" + half.Length + " addr)");
            MeasureContinuous(chanId, dAddrs.ToArray(), "default profile (" + dAddrs.Count + " addr)");
            int big = Math.Min(g_maxBatch > 0 ? g_maxBatch : 40, flat.Count);
            if (big > dAddrs.Count)
                MeasureContinuous(chanId, flat.GetRange(0, big).ToArray(), "full batch (" + big + " addr)");

            // (2) sustained stability over a longer run — the firmware's real logging path
            SustainedStreamTest(chanId, dAddrs.ToArray(), 10.0);
        }
        else
            Log("    (no default-profile params supported by this ECU — skipping rate/stability tests)");

        // (3) full decoded snapshot (raw + engineering value), chunked by the probed ceiling
        Log("");
        Log("── Full ECU snapshot — all supported P-params (raw + decoded) ───────");
        if (keys.Count == 0) { Log("    (none reported supported)"); return; }
        byte[] vals = new byte[flat.Count];
        int batch = g_maxBatch > 0 ? g_maxBatch : 32;
        long t0 = Stopwatch.GetTimestamp();
        bool ok = true;
        for (int o = 0; o < flat.Count && ok; o += batch)
        {
            int cnt = Math.Min(batch, flat.Count - o);
            var chunk = new byte[cnt][];
            for (int i = 0; i < cnt; i++) chunk[i] = flat[o + i];
            byte[] d = ReadAddrs(chanId, chunk);
            if (d == null) { ok = false; break; }
            for (int i = 0; i < cnt; i++) vals[o + i] = d[i];
        }
        double readMs = Ticks2Ms(Stopwatch.GetTimestamp() - t0);
        if (!ok) { Log("    snapshot read failed."); return; }
        for (int i = 0; i < keys.Count; i++)
        {
            List<SupParam> g = byKey[keys[i]];
            SupParam p0 = g[0];
            long raw = 0;
            for (int k = 0; k < p0.len; k++) raw = (raw << 8) | vals[off[i] + k];
            string rawHex = "";
            for (int k = 0; k < p0.len; k++) rawHex += (k > 0 ? " " : "") + vals[off[i] + k].ToString("X2");
            if (g.Count == 1)
            {
                List<string> wl = WrapName(p0.name, 36);   // wrap a long description onto extra lines
                Log(string.Format("    {0,-6} {1,-36} {2,11} {3,-10} raw {4}",
                    p0.id, wl[0], DecodeVal(p0, raw), p0.units, rawHex));
                for (int w2 = 1; w2 < wl.Count; w2++) Log("           " + wl[w2]);
            }
            else
            {
                // address collision: one byte, multiple defs interpretations — show all, labeled
                string ids = "", variants = "";
                foreach (SupParam p in g)
                {
                    ids += (ids.Length > 0 ? "/" : "") + p.id;
                    string tag = ParenTag(p.name);
                    if (tag.Length == 0) tag = p.id;   // no parenthetical → label by id
                    variants += (variants.Length > 0 ? "  |  " : "") + tag + "=" + DecodeVal(p, raw) + " " + p.units;
                }
                Log(string.Format("    {0,-6} {1,-36} raw {2}", ids, Trunc(BaseName(p0.name), 36), rawHex));
                Log("           " + variants + "   (same byte — pick the variant matching your ECU/hardware)");
            }
        }
        Log(string.Format("    -> {0} params / {1} unique addresses in {2}-addr chunks, full read in {3:F0} ms.", sup.Count, flat.Count, batch, readMs));
    }

    // Assemble + decode one param's raw value via its conversion formula (signed-aware).
    static string DecodeVal(SupParam p, long raw)
    {
        double x = raw;
        if (p.signed) { int bits = p.len * 8; if ((raw & (1L << (bits - 1))) != 0) x = raw - (1L << bits); }
        double v = EvalExpr(p.expr, x);
        return double.IsNaN(v) ? ("0x" + raw.ToString("X")) : v.ToString("0.###");
    }
    // Text inside the first (...) of a name: "A/F Correction #3 (16-bit ECU)" -> "16-bit ECU".
    static string ParenTag(string name)
    {
        if (name == null) return "";
        int a = name.IndexOf('('), b = name.IndexOf(')');
        return (a >= 0 && b > a) ? name.Substring(a + 1, b - a - 1) : "";
    }
    static string BaseName(string name)
    {
        if (name == null) return "?";
        int a = name.IndexOf('(');
        return (a > 0) ? name.Substring(0, a).Trim() : name;
    }
    // Word-wrap a description to width w; returns >=1 lines (long names span multiple rows).
    static List<string> WrapName(string name, int w)
    {
        var outl = new List<string>();
        if (string.IsNullOrEmpty(name)) { outl.Add(""); return outl; }
        string cur = "";
        foreach (string wd in name.Split(' '))
        {
            if (cur.Length == 0) cur = wd;
            else if (cur.Length + 1 + wd.Length <= w) cur += " " + wd;
            else { outl.Add(cur); cur = wd; }
        }
        if (cur.Length > 0) outl.Add(cur);
        if (outl.Count == 0) outl.Add("");
        return outl;
    }

    // Stream `addrs` continuously for `seconds`; report frames OK, bad-checksum, dropped, jitter.
    static void SustainedStreamTest(uint chanId, byte[][] addrs, double seconds)
    {
        Log("");
        Log(string.Format("── Sustained streaming stability ({0:F0} s — firmware's logging path) ──", seconds));
        if (addrs.Length == 0) { Log("    (no default params)"); return; }
        Send(chanId, BuildBatchRead(addrs, true));
        long t0 = Stopwatch.GetTimestamp(), prev = 0;
        int good = 0, bad = 0, drop = 0, periods = 0;
        double sum = 0, sumSq = 0, min = 1e9, max = 0; bool first = true;
        var gaps = new List<double>();   // every inter-frame gap, for the percentile/timeout analysis
        while (Ticks2Ms(Stopwatch.GetTimestamp() - t0) < seconds * 1000.0)
        {
            byte[] r = Recv(chanId, 1500);
            long t = Stopwatch.GetTimestamp();
            if (r == null) { drop++; continue; }
            if (!ValidFrame(r, addrs.Length)) { bad++; continue; }
            good++;
            if (!first) { double dt = Ticks2Ms(t - prev); sum += dt; sumSq += dt * dt; if (dt < min) min = dt; if (dt > max) max = dt; periods++; gaps.Add(dt); }
            prev = t; first = false;
        }
        Send(chanId, BuildBatchRead(addrs, false));
        for (int k = 0; k < 8; k++) { if (Recv(chanId, 150) == null) break; }
        int total = good + bad + drop;
        Log("    frames OK:        " + good);
        Log("    bad checksum:     " + bad);
        Log("    dropped/timeout:  " + drop);
        if (periods > 0)
        {
            double mean = sum / periods;
            double vr = sumSq / periods - mean * mean;
            double sd = vr > 0 ? Math.Sqrt(vr) : 0;
            Log(string.Format("    period: mean {0:F1} ms ({1:F0} Hz)  min {2:F1}  max {3:F1}  jitter(sd) {4:F1} ms", mean, 1000.0 / mean, min, max, sd));
            if (gaps.Count >= 10)
            {
                gaps.Sort();
                double p50 = gaps[gaps.Count / 2];
                double p90 = gaps[(int)(gaps.Count * 0.90)];
                double p99 = gaps[Math.Min(gaps.Count - 1, (int)(gaps.Count * 0.99))];
                Log(string.Format("    gap percentiles: p50 {0:F1}  p90 {1:F1}  p99 {2:F1}  max {3:F1} ms", p50, p90, p99, max));
                Log(string.Format("    ► firmware frame timeout ≈ 2× max gap = {0:F0} ms (gap beyond this = stream lost, re-init)", 2 * max));
            }
            Log(string.Format("    -> {0} good frames, {1:F2}% dropped/bad over {2:F0} s.", good, total > 0 ? 100.0 * (bad + drop) / total : 0.0, seconds));
        }
        else Log("    did not stream (no good frames).");
    }

    // Validate a continuous (0xA8) SSM response frame: header, length, cmd 0xE8, checksum.
    static bool ValidFrame(byte[] r, int nAddr)
    {
        if (r == null || r.Length < 6) return false;
        if (r[0] != 0x80 || r[1] != 0xF0 || r[2] != 0x10) return false;
        int len = r[3];
        if (r.Length != 5 + len || r[4] != 0xE8 || len != nAddr + 1) return false;
        int cs = 0; for (int i = 0; i < r.Length - 1; i++) cs += r[i];
        return (byte)cs == r[r.Length - 1];
    }

    class SupParam { public string id, name, addrHex, expr, units = ""; public int len = 1; public bool signed; }

    // Enumerate capability-SUPPORTED P-params from the SSM section. Collects ALL conversions and
    // picks the first AMERICAN/imperial one (so temps read °F, pressures psi, speed mph, etc.);
    // falls back to the first conversion when no imperial variant exists (e.g. g/s, %, V, rpm).
    static List<SupParam> EnumSupportedParams(string[] lines, byte[] initResp)
    {
        var list = new List<SupParam>();
        SupParam cur = null;
        var cExpr = new List<string>(); var cUnits = new List<string>(); var cSigned = new List<bool>();
        foreach (string line in lines)
        {
            if (line.IndexOf("</protocol>") >= 0) break;          // SSM section only
            if (cur == null)
            {
                if (line.IndexOf("<parameter ") < 0) continue;
                string bi = Attr(line, "ecubyteindex"), bt = Attr(line, "ecubit");
                if (bi == null || bt == null) continue;            // only capability-gated P-params
                int byteIdx, b;
                if (!int.TryParse(bi, out byteIdx) || !int.TryParse(bt, out b)) continue;
                int idx = 5 + byteIdx;
                if (!(initResp != null && idx < initResp.Length && (initResp[idx] & (1 << b)) != 0)) continue;  // supported only
                cur = new SupParam(); cur.id = Attr(line, "id"); cur.name = Attr(line, "name") ?? "?";
                cExpr.Clear(); cUnits.Clear(); cSigned.Clear();
            }
            else
            {
                if (cur.addrHex == null && line.IndexOf("<address") >= 0)
                {
                    string lenS = Attr(line, "length"); if (lenS != null) int.TryParse(lenS, out cur.len);
                    int gt = line.IndexOf('>', line.IndexOf("<address"));
                    int lt = gt >= 0 ? line.IndexOf('<', gt + 1) : -1;
                    if (gt >= 0 && lt > gt) cur.addrHex = line.Substring(gt + 1, lt - gt - 1).Trim();
                }
                else if (line.IndexOf("<conversion ") >= 0)
                {
                    cExpr.Add(Attr(line, "expr"));
                    cUnits.Add(Attr(line, "units") ?? "");
                    string st = Attr(line, "storagetype");
                    cSigned.Add(st != null && st.StartsWith("int"));
                }
                if (line.IndexOf("</parameter>") >= 0)
                {
                    if (cur.addrHex != null && cExpr.Count > 0)
                    {
                        int pick = 0;                                  // prefer the first AMERICAN/imperial conversion
                        for (int j = 0; j < cUnits.Count; j++) { if (IsImperial(cUnits[j])) { pick = j; break; } }
                        cur.expr = cExpr[pick]; cur.units = cUnits[pick]; cur.signed = cSigned[pick];
                        if (cur.expr != null) list.Add(cur);
                    }
                    cur = null;
                }
            }
        }
        return list;
    }

    // American/imperial units we PREFER when the param offers them (snapshot reads US-customary).
    static bool IsImperial(string u)
    {
        if (u == null) return false;
        string s = u.Trim().ToLowerInvariant();
        return s == "mph" || s == "psi" || s == "f" || s == "°f" || s == "mi" || s == "mile" || s == "miles" || s == "inhg";
    }

    // Minimal arithmetic evaluator for P-param conversion formulas: + - * / % () , x , decimals.
    static int _ep; static string _es;
    static double EvalExpr(string expr, double x)
    {
        if (string.IsNullOrEmpty(expr)) return x;
        _es = expr.Replace(" ", ""); _ep = 0;
        try { return EParseE(x); } catch { return double.NaN; }
    }
    static double EParseE(double x)
    {
        double v = EParseT(x);
        while (_ep < _es.Length && (_es[_ep] == '+' || _es[_ep] == '-'))
        { char op = _es[_ep++]; double t = EParseT(x); v = (op == '+') ? v + t : v - t; }
        return v;
    }
    static double EParseT(double x)
    {
        double v = EParseF(x);
        while (_ep < _es.Length && (_es[_ep] == '*' || _es[_ep] == '/' || _es[_ep] == '%'))
        { char op = _es[_ep++]; double f = EParseF(x); v = (op == '*') ? v * f : (op == '/') ? v / f : v % f; }
        return v;
    }
    static double EParseF(double x)
    {
        if (_ep >= _es.Length) throw new Exception();
        char c = _es[_ep];
        if (c == '-') { _ep++; return -EParseF(x); }
        if (c == '+') { _ep++; return EParseF(x); }
        if (c == '(') { _ep++; double v = EParseE(x); if (_ep < _es.Length && _es[_ep] == ')') _ep++; return v; }
        if (c == 'x' || c == 'X') { _ep++; return x; }
        int s = _ep;
        while (_ep < _es.Length && (char.IsDigit(_es[_ep]) || _es[_ep] == '.')) _ep++;
        if (_ep == s) throw new Exception();
        return double.Parse(_es.Substring(s, _ep - s), System.Globalization.CultureInfo.InvariantCulture);
    }

    // Stream a continuous 0xA8 read of the given addresses; report the inter-response period.
    static void MeasureContinuous(uint chanId, byte[][] addrs, string label)
    {
        int n = addrs.Length;
        Send(chanId, BuildBatchRead(addrs, true));
        int got = 0; long prev = 0; double sum = 0, min = 1e9, max = 0;
        for (int i = 0; i < 14; i++)
        {
            byte[] r = Recv(chanId, 1500);
            long t = Stopwatch.GetTimestamp();
            if (r == null) break;
            if (got > 0) { double dt = Ticks2Ms(t - prev); sum += dt; if (dt < min) min = dt; if (dt > max) max = dt; }
            prev = t; got++;
        }
        // Stop the stream + drain any buffered responses before the next scenario.
        Send(chanId, BuildBatchRead(addrs, false));
        for (int k = 0; k < 6; k++) { if (Recv(chanId, 150) == null) break; }
        if (got >= 3)
        {
            double mean = sum / (got - 1);
            Log(string.Format("    {0,-29} {1,3}   {2,6:F1} ms  {3,4:F0} Hz  (min {4:F1}/max {5:F1})",
                label, n, mean, 1000.0 / mean, min, max));
        }
        else
            Log(string.Format("    {0,-29} {1,3}   did not stream", label, n));
    }

    static uint HexU(string hex)
    {
        hex = hex.Trim();
        if (hex.StartsWith("0x") || hex.StartsWith("0X")) hex = hex.Substring(2);
        return Convert.ToUInt32(hex, 16);
    }

    static bool BitSet(List<uint> distinct, byte[] val, uint addr, int bit)
    {
        if (addr == 0xFFFFFFFF) return false;
        int idx = distinct.IndexOf(addr);
        return idx >= 0 && (val[idx] & (1 << bit)) != 0;
    }

    // Read & list DTCs at end-of-run. Each <dtcode> has tmpaddr (current/pending) + memaddr
    // (stored) flag bytes + a bit. Reads the distinct flag addresses via the continuous path
    // (chunked to the batch limit), decodes set bits -> code names, and TIMES the whole read
    // (one-time cost a firmware DTC-read would add at logging startup; constant vs #codes).
    static void DtcCheck(uint chanId, string defsPath, int ecuInitLength)
    {
        Log("");
        Log("── Diagnostic trouble codes (DTCs) ──────────────────────────────────");
        if (defsPath == null || !File.Exists(defsPath)) { Log("    (no defs — skipping)"); return; }

        // RomRaider ReadCodesManagerImpl: read ONLY the SSM <dtcodes> section, and trim the
        // list by init length — stop at D488 when ecuInitLength<104 (gasoline subset). This
        // avoids reading unmapped (diesel/6cyl/CAN) addresses that read 0xFF → false codes.
        string lastCode = (ecuInitLength < 104) ? "D488" : null;
        var name = new List<string>(); var tmpA = new List<uint>();
        var memA = new List<uint>();   var bit  = new List<int>();
        foreach (string line in File.ReadAllLines(defsPath))
        {
            if (line.IndexOf("</protocol>") >= 0) break;            // SSM section only (it's first)
            if (line.IndexOf("<dtcode ") < 0) continue;
            string id = Attr(line, "id");
            if (lastCode != null && id == lastCode) break;          // RomRaider trim
            string ts = Attr(line, "tmpaddr"), ms = Attr(line, "memaddr"), bs = Attr(line, "bit");
            if (ts == null || bs == null) continue;
            int b; if (!int.TryParse(bs, out b)) continue;
            name.Add(Attr(line, "name") ?? "?");
            tmpA.Add(HexU(ts));
            memA.Add(ms != null ? HexU(ms) : 0xFFFFFFFF);
            bit.Add(b);
        }
        if (name.Count == 0) { Log("    (no <dtcode> entries in defs)"); return; }

        var distinct = new List<uint>();
        foreach (uint a in tmpA) if (a != 0xFFFFFFFF && !distinct.Contains(a)) distinct.Add(a);
        foreach (uint a in memA) if (a != 0xFFFFFFFF && !distinct.Contains(a)) distinct.Add(a);

        // Read all distinct flag addresses (chunked to a safe batch size), timing it.
        var val = new byte[distinct.Count];
        long t0 = Stopwatch.GetTimestamp();
        int CHUNK = g_maxBatch > 0 ? g_maxBatch : 32;   // ECU's probed batch ceiling
        bool ok = true;
        for (int off = 0; off < distinct.Count && ok; off += CHUNK)
        {
            int cnt = Math.Min(CHUNK, distinct.Count - off);
            var addrs = new byte[cnt][];
            for (int i = 0; i < cnt; i++)
            {
                uint a = distinct[off + i];
                addrs[i] = new byte[] { (byte)(a >> 16), (byte)(a >> 8), (byte)a };
            }
            byte[] d = ReadAddrs(chanId, addrs);
            if (d == null) { ok = false; break; }
            for (int i = 0; i < cnt; i++) val[off + i] = d[i];
        }
        double readMs = Ticks2Ms(Stopwatch.GetTimestamp() - t0);
        if (!ok) { Log("    DTC read failed."); return; }

        int nCur = 0, nSto = 0;
        var sb = new StringBuilder();
        for (int i = 0; i < name.Count; i++)
        {
            bool cur = BitSet(distinct, val, tmpA[i], bit[i]);
            bool sto = BitSet(distinct, val, memA[i], bit[i]);
            if (!cur && !sto) continue;
            if (cur) nCur++; if (sto) nSto++;
            sb.Append("    ").Append(name[i])
              .Append(cur && sto ? "  [current+stored]" : cur ? "  [current]" : "  [stored]").Append('\n');
        }
        Log(nCur + nSto == 0 ? "    none" : sb.ToString().TrimEnd());
        Log(string.Format("    -> {0} current, {1} stored; {2} flag addresses read in {3:F0} ms (constant vs #codes).",
            nCur, nSto, distinct.Count, readMs));
    }

    // ── SSM packet builders ──────────────────────────────────────────────────

    // Init: 80 10 F0 01 BF <cs>
    static byte[] BuildInit() { return BuildInitDst(0x10); }   // 0xBF init to the ECU

    // Init (0xBF) to an arbitrary module: 0x10 = ECU, 0x18 = TCU / transmission controller.
    static byte[] BuildInitDst(byte dst)
    {
        byte[] pkt = new byte[6];
        pkt[0] = 0x80; pkt[1] = dst; pkt[2] = 0xF0; pkt[3] = 0x01; pkt[4] = 0xBF;
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
        if (g_serial) { SerialSend(data); return; }
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

    // Serial (generic K-line cable) transport: half-duplex, software framing.
    static void SerialSend(byte[] data)
    {
        try { g_port.DiscardInBuffer(); } catch { }
        g_port.Write(data, 0, data.Length);
        // Discard the half-duplex echo (our own TX loops back on RX), like the firmware does.
        g_port.ReadTimeout = 600;
        for (int i = 0; i < data.Length; i++)
        {
            try { g_port.ReadByte(); } catch (TimeoutException) { break; }
        }
    }

    // Read one framed SSM response (80 F0 10 <len> <len+1 bytes>); resync to 0x80. Null on timeout.
    static byte[] SerialRecv(int timeoutMs)
    {
        if (timeoutMs < 1) timeoutMs = 1;
        try
        {
            g_port.ReadTimeout = timeoutMs;
            int first;
            do { first = g_port.ReadByte(); } while (first != 0x80);   // sync to frame start
            g_port.ReadTimeout = 300;                                  // rest of frame comes fast
            int f0 = g_port.ReadByte(), mod = g_port.ReadByte(), len = g_port.ReadByte();
            if (f0 != 0xF0 || (mod != 0x10 && mod != 0x18)) return null;   // 0x10 = ECU, 0x18 = TCU
            byte[] frame = new byte[5 + len];
            frame[0] = 0x80; frame[1] = 0xF0; frame[2] = (byte)mod; frame[3] = (byte)len;
            for (int i = 0; i < len + 1; i++) frame[4 + i] = (byte)g_port.ReadByte();
            return frame;
        }
        catch (Exception) { return null; }   // TimeoutException or port error
    }

    // Returns first non-echo message within timeoutMs, or null
    static byte[] Recv(uint chanId, uint timeoutMs)
    {
        if (g_serial) return SerialRecv((int)timeoutMs);
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
