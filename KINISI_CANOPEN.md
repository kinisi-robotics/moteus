# Kinisi CANopen port — plan & progress

Living document. **Update the checklist as work proceeds** so any session (human or
Claude) can resume mid-stream. Lives on the `kinisi/canopen` branch of
`github.com/kinisi-robotics/moteus` (fork of `mjbots/moteus`).

## Goal

Make a moteus-n1 join Kinisi's classic CAN 2.0 buses (1 Mbps, 11-bit IDs) as a
CANopen node speaking the **Kinisi actuator profile**, driven by the custom
Lely-core master in `kinisi_ros` (`kinisi_canbus_hardware`), exactly like the
existing gripper nodes from `kinisi_embedded`.

## Decisions (locked)

- **Stack**: port CANopenNode v4 (commit `6dfd4ed7f22ae4b23097cf76c66a909ace5f3622`,
  v4.0-389 — the exact commit vendored in `kinisi_embedded`), Apache-2.0.
- **Profile**: Kinisi custom actuator profile OD 0x6000–0x601E (NOT CiA 402).
- **Dual-mode**: persisted `can.mode` config selects at boot between stock
  CAN-FD/mjlib protocol (calibration/tview/flash) and classic-CAN CANopen.
- **Repo**: kinisi-robotics org fork, branch `kinisi/canopen`, Bazel build kept
  intact, upstream `mjbots/moteus` as `origin` for merges. Remote name `kinisi`.
- **Runtime use**: position control @ master's 100 Hz SYNC (commandPosition +
  commandVelocity feedforward + commandCurrent), like the grippers.
- **Build env**: WSL Ubuntu-24.04 on this machine (`tools/bazel`), Windows checkout
  at `C:\Users\paris\Documents\open_source\moteus`.

## Key architecture facts (from investigation, 2026-09-01)

### moteus side
- `fw/fdcan.h` `FDCan::Options`: `fdcan_frame=false` + `bitrate_switch=false` →
  `FDCAN_FRAME_CLASSIC` (fw/fdcan.cc:122-129). `slow_bitrate=1000000` for 1 Mbps.
  Classic CAN already fully supported at driver level; currently hardcoded to FD
  in `fw/moteus.cc` (~line 206-227, FDCan options lambda).
- DLC: `RoundUpDlc` (fw/fdcan.cc:24) supports FD sizes; classic mode must cap ≤8B.
- Protocol seam: `FDCan` → `FDCanMicroServer` (fw/fdcan_micro_server.h) →
  `MultiTransportDatagramServer` → mjlib `MicroServer` → `MoteusController::Impl`
  register switch (fw/moteus_controller.cc:601+). CANopen bypasses all of this.
- Control API: `BldcServo::Command(CommandData&)` / `status()` / `config()`
  (fw/bldc_servo.h). `MoteusController::bldc_servo()` exposes it
  (fw/moteus_controller.h:53). Units: revolutions, rev/s, Nm.
- Config: `CanConfig` struct + `persistent_config.Register("can", ...)` in
  fw/moteus.cc (~141-152, 338). Extend this for `mode` etc.
- Main loop: cooperative `for(;;)` polling + 1 ms tick (fw/moteus.cc:348-376).
  FOC loop runs in ADC ISR, decoupled. Static pool `SizedPool<24000>` — watch RAM.
- One FDCAN peripheral → native-FD and CANopen-classic are boot-time either/or.
- n1 = hardware family 1, runtime-detected; single firmware image for all boards.
- Build: `tools/bazel build --config=target //:target`; host tests
  `tools/bazel test --config=host //:host`. Flash: `//fw:flash` (openocd/stlink)
  or `moteus_tool --flash` over CAN (native mode only).

### Kinisi side (what the node must implement)
- Master (kinisi_ros `kinisi_canbus_hardware`, Lely-core `OpencanMaster`):
  NMT (RESET_COMM broadcast at init, per-node RESET_NODE retries, START/STOP),
  heartbeat producer 1000 ms + consumes slave heartbeat (1500 ms timeout),
  boot-time SDO writes that **dynamically configure slave PDO comm+mapping**
  (0x1400-3/0x1600-3/0x1800-3/0x1A00-3), SYNC @ 100 Hz COB 0x80, PDOs
  transmission-type 1 (synchronous), EMCY logged. Identity check: 0x1018 sub1
  vendor = 0x3000 (Kinisi), sub2 product code per device type.
- Kinisi actuator profile (see kinisi_embedded `dynamixel-controller/ObjectDictionary/OD.h`):
  - 0x6000 updateMode u8 RW; 0x6001 commandPosition f32 **deg** RPDO;
    0x6002 commandVelocity f32 **RPM** RPDO; 0x6003 commandCurrent f32 **mA** RPDO;
    0x6004/5/6 position/velocity/current feedback (same units) TPDO;
    0x6007 controlWord u8 RPDO (bit0=torque enable, bit7=reboot);
    0x6008 statusWord u8 TPDO (bit0=torque on, bit1=error, bit2=comms error);
    0x6009 errorCode (i32 OD / u8 on PDO) TPDO; 0x6010 maxVelocity u32;
    0x6011 nodeID u8 RW **flash-persisted, SDO-writable**; 0x6012 controlMode u8;
    0x6014 currentLimit f32 mA; 0x6015 temperature f32; 0x6016/0x6017
    productCode/vendorId u32; 0x601A persistTrigger u8; 0x601E hardwareRevision u32.
  - PDO defaults EMPTY mapping (`numberOfMapped...=0`) — master maps at boot.
  - RPDO payload 13 B / TPDO 14 B → 2 PDOs per direction (classic 8 B frames).
  - Standard COB-IDs: TPDOn 0x180/0x280/…+ID, RPDOn 0x200/0x300/…+ID,
    SDO 0x600/0x580+ID, HB 0x700+ID, EMCY 0x80+ID.
- Slave firmware reference: kinisi_embedded uses CANopenNode v4 + CanOpenNodeSTM32
  (`CO_driver_STM32.c` — STM32 FDCAN HAL port, Apache-2.0), 1 SDO server,
  4 RPDO + 4 TPDO, heartbeat 500 ms first / 1000 ms period, NMT_CONTROL =
  CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION.
- Node IDs in use: 18 = gripper (each bus), 34 = head LED (can0). Check live
  robot's `combined.yaml` ($KINISI_YAML_PATH) before picking. Provisioning via
  FirmwareTool `set_nodeid.py` (SDO 0x6011) / LSS for unconfigured.
- ROS driver to add later (Phase 4): `MoteusCan` in kinisi_canbus_hardware,
  mirror `zlac_can.hpp`/`dynamixel_can.hpp` + `config/Moteus/{data,pdos,sdos}.hpp`
  + xacro; type dispatch is hardcoded in `canopen_hardware_interface.cpp`.

## Open decisions

- [ ] `commandCurrent` (0x6003, mA) semantics: q-axis current in mA vs torque.
      Leaning: treat as feedforward torque converted via motor Kt (moteus native
      is Nm). DECIDE AT PHASE 3, ask Paris.
- [ ] Product code for "moteus joint" (grippers use 0x10/0x11/0x12; QVC=2, LED=3).
      Ask Paris / check kinisi conventions. Placeholder: 0x20.
- [ ] Node ID for the new device (free: most of 1-127 except 18, 34 + live overlay).
- [ ] Whether UART diagnostic path (tview over aux UART) stays active in CANopen
      mode — decide when wiring main().

## Phase checklist

### Phase 0 — fork setup ✅ DONE (2026-09-01)
- [x] Fork created: `github.com/kinisi-robotics/moteus`
- [x] Remote `kinisi` added, branch `kinisi/canopen` created
- [x] This plan file committed
- [ ] Branch pushed to kinisi remote (push after each work session)

### Phase 1 — classic-CAN plumbing + mode switch (firmware) — IN PROGRESS
- [ ] Add `mode` field (0=fd-native, 1=classic-canopen) + `canopen_node_id` to
      `CanConfig` in fw/moteus.cc, persisted, with Serialize/MJ_NVP
- [ ] Thread into `FDCan::Options` construction (classic: fdcan_frame=false,
      bitrate_switch=false, slow_bitrate=1e6, automatic_retransmission=true)
- [ ] Guard DLC ≤8 in classic mode (fdcan.cc Send / RoundUpDlc path)
- [ ] Firmware builds clean in WSL (`--config=target`)

### Phase 2 — CANopenNode port
- [ ] Vendor CANopenNode @ 6dfd4ed into `fw/canopen/CANopenNode/` (in-tree copy,
      keep LICENSE, note commit in a VENDORED.md) + Bazel BUILD (cc_library)
- [ ] `CO_driver_target.h` + CO driver bridge: RX/TX via existing `FDCan`
      (polled from main loop), 1 ms processing from millisecond tick, no RTOS,
      CO_CONFIG trimmed to: NMT slave, HB producer, 1 SDO server, SYNC consumer,
      4 RPDO + 4 TPDO, EMCY producer. No LSS initially (FirmwareTool uses it only
      for unconfigured nodes; node ID comes from persisted config).
- [ ] OD.h/OD.c: CiA 301 mandatory (0x1000/0x1001/0x1005/0x1014/0x1017/0x1018)
      + PDO comm/mapping 0x1400-3/0x1600-3/0x1800-3/0x1A00-3 (empty default maps)
      + Kinisi profile 0x6000-0x601E. Adapt from kinisi_embedded
      dynamixel-controller OD. Identity: vendor 0x3000, product code TBD.
- [ ] Host-side unit test: replay master's boot SDO sequence (PDO remap) against
      OD + SDO server, assert mapping accepted (bazel host test)

### Phase 3 — profile glue (CanopenServer)
- [ ] `fw/canopen_server.h/.cc`: constructed in main() when can.mode==canopen;
      owns CO_t instance; on SYNC latch RPDO → unit-convert (deg→rev, RPM→rev/s,
      mA→torque via Kt or q-current — see open decision) → BldcServo::Command()
      position mode; controlWord bit0 gates kPosition/kStopped; status() → TPDO
      (rev→deg etc.); moteus faults → statusWord bits + errorCode + EMCY;
      command timeout as comms-loss safety
- [ ] 0x6011 SDO write → persist node id via PersistentConfig (+ 0x601A persist
      trigger), reboot bit7 of controlWord
- [ ] main() wiring: in canopen mode skip/bypass FDCanMicroServer CAN transport
      (decide UART diag path), install standard-11-bit filters for
      NMT/SYNC/SDO/RPDO COB-IDs
- [ ] Host tests for unit conversion + controlWord/status mapping
- [ ] Full firmware builds; RAM/flash budget checked (SizedPool<24000>)

### Phase 4 — ROS driver (kinisi_ros)
- [ ] `MoteusCan` driver + config headers + xacro + type dispatch (mirror ZLAC/
      Dynamixel patterns). Compile-test only until bench.

### Phase 5 — bench bring-up (needs hardware)
- [ ] n1 + SWD probe + socketcan adapter @ 1 Mbps classic
- [ ] Flash, flip can.mode, verify frames/heartbeat with python-canopen
- [ ] FirmwareTool scan + set_nodeid
- [ ] Lely master boot sequence (PDO remap SDOs) succeeds; 100 Hz SYNC PDO exchange
- [ ] Motor position control via ros2_control; coexist with gripper node on bench bus
- [ ] Native-mode regression: calibrate with moteus_tool over fdcanusb still works

## Resume notes

- Working dir: `C:\Users\paris\Documents\open_source\moteus`, branch `kinisi/canopen`.
- Reference trees on this machine: `C:\Users\paris\Documents\kinisi_embedded`
  (CANopenNode submodule, gripper ODs, CanOpenNodeSTM32), `C:\Users\paris\Documents\kinisi_ros`
  (master, `kinisi_canbus_hardware`).
- Build from WSL Ubuntu-24.04: repo visible at `/mnt/c/Users/paris/Documents/open_source/moteus`
  (consider cloning inside WSL FS if /mnt/c bazel perf is bad).
- Claude memory for this project also stores the architecture summary
  (`~/.claude/projects/C--Users-paris-Documents-open-source-moteus/memory/`).
