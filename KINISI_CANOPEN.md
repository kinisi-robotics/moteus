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

## Decisions made during implementation (review with Paris)

- **Torque semantics (v1)**: for the moteus device type, 0x6003 commandCurrent
  carries feedforward TORQUE in Nm, 0x6006 current feedback carries measured
  torque in Nm, 0x6014 currentLimit carries max torque in Nm.  Wire type (f32)
  unchanged from the gripper profile; only the unit interpretation differs.
  Rationale: moteus is natively torque-based (Nm); avoids needing motor Kt in
  the protocol layer; matches the ros2_control effort interface directly.
  Trivial to change to mA later if preferred.
- Product code 0x20 (placeholder, in OD 0x1018 sub2 + 0x6016), vendor 0x3000.
- Command watchdog: 0.25 s (servo stops if SYNC/commands cease).
- controlWord bit0 = torque enable (kPosition), bit7 = reboot.  statusWord
  bit0 = torque on, bit1 = fault.  errorCode = moteus errc fault code.  EMCY
  (generic error, moteus fault code in info bytes) emitted on fault transitions.
- LSS slave kept enabled (matches grippers / FirmwareTool fast-scan).
- CANopen persist: SDO write 0x6011 sets can.canopen_id; SDO write nonzero to
  0x601A injects "conf write" into the console command stream (whole config is
  persisted).  Refused while the servo is enabled (flash write stalls the CPU).
- UART diagnostics (tview/moteus_tool over aux UART fdcanusb emulation) REMAIN
  ACTIVE in CANopen mode; only the CAN multiplex transport is disabled.
- Default node id 127 (can.canopen_id), clamped to [1,127].
- 0x601D imuEnabled dropped from the OD (moteus has no IMU); everything else
  in 0x6000-0x601E kept verbatim.

## Open items

- [ ] Confirm torque-in-Nm semantics with Paris (see above).
- [ ] Pick real product code + node ID (free: most of 1-127 except 18, 34 +
      whatever the live robot's combined.yaml overlay uses).
- [ ] Flash headroom: app is 443KB of the 454KB app region (baseline 419KB;
      CANopen adds ~24KB).  If upstream merges outgrow the region, trim
      CANopenNode config (TIME, HB consumer, LEDs, node guarding) via
      CO_driver_target.h defines.
- [ ] WSL host-test env: //lib/python:bdist_wheel genrule fails (missing
      python3 build deps in WSL) - pre-existing env issue; fw C++ tests pass.

## Phase checklist

### Phase 0 — fork setup ✅ DONE (2026-09-01)
- [x] Fork created: `github.com/kinisi-robotics/moteus`
- [x] Remote `kinisi` added, branch `kinisi/canopen` created
- [x] This plan file committed
- [x] Branch pushed to kinisi remote (push after each work session)

### Phase 1 - classic-CAN plumbing + mode switch -- DONE 2026-09-01
- [x] can.mode (0=fd-native, 1=classic-canopen) + can.canopen_id persisted
      config; applied via maybe_update_filters (FDCan::Reconfigure at load)
- [x] Classic FDCan options (fdcan_frame=false, bitrate_switch=false, 1 Mbps);
      accept-all-standard / reject-extended filters in CANopen mode
- [x] DLC <=8 guard in classic mode (fdcan.cc MakeTxHeader); FDCan::TrySend +
      TxQueueFree added (non-aborting TX needed for CANopen bursts)
- [x] multi_transport.SetCanDisabled() - CANopen owns the peripheral, UART
      diagnostics stay up
- [x] Firmware builds clean in WSL (tools/bazel build --config=target //fw:moteus)

### Phase 2 - CANopenNode port -- DONE 2026-09-01
- [x] Vendored CANopenNode @ 6dfd4ed into fw/canopen/CANopenNode/ (+ extra/,
      storage/; VENDORED.md notes commit; Bazel cc_library //fw:canopen_node)
- [x] fw/canopen/CO_driver_target.h (CO_USE_GLOBALS, recursive irq locks) +
      fw/canopen/canopen_driver.cc: poll-based CO driver over FDCan (software
      TX queue flushed by CanopenPoll, bus-off recovery in CO_CANmodule_process).
      Stock CO_CONFIG defaults kept (matches grippers); LSS slave INCLUDED.
- [x] fw/canopen/ObjectDictionary/OD.{h,c}: 58 entries - CiA 301 comm objects,
      4 RPDO / 4 TPDO with empty default mappings, full Kinisi profile
      0x6000-0x601E (minus 0x601D imuEnabled).  Identity vendor 0x3000 /
      product 0x20.  gcc-verified + OD_find cross-check.
- [ ] Host-side SDO-replay unit test (deferred - bench bring-up exercises the
      PDO remap handshake; add if bring-up hits protocol issues)

### Phase 3 - profile glue (CanopenServer) -- DONE 2026-09-01
- [x] fw/canopen/canopen_server.{h,cc}: SYNC-driven RPDO -> unit conversion
      (deg->rev, RPM->rev/s, Nm feedforward) -> BldcServo::Command() position
      mode; controlWord gating; feedback TPDO; faults -> statusWord/errorCode/
      EMCY; 0.25 s command watchdog
- [x] 0x6011 node-id SDO extension -> can.canopen_id; 0x601A persist trigger ->
      injects "conf write" via fw/canopen/injectable_command_stream.h;
      controlWord bit7 + NMT reset-app -> NVIC_SystemReset
- [x] main() wiring: CanopenServer constructed always, Start()ed only in
      canopen mode; polled from the main loop
- [ ] Host tests for unit conversion + controlWord/status mapping (deferred,
      same rationale as the Phase 2 test)
- [x] Full firmware builds; fw host tests pass; app 443KB of 454KB region
      (CO_USE_GLOBALS statics ~3KB RAM; SizedPool untouched)

### Phase 4 - ROS driver (kinisi_ros) -- CODE DONE 2026-09-01 (uncompiled)
- [x] kinisi_ros branch moteus-canopen-driver, commit 0bb5c1d734 (NOT pushed)
- [x] MoteusCan driver (moteus_can.hpp), MoteusControl (devices/moteus.hpp),
      config/Moteus/{data,pdos,sdos,types,moteus_canbus}.hpp,
      urdf/hardware/moteus.ros2_control.xacro (macro "Moteus", type "moteus"),
      wired through all 10 dispatch sites in canopen_hardware_interface,
      +7 tests in test_canopen_migration.cpp
- [x] Interfaces: cmd position/velocity/effort/control_word; state position/
      velocity/effort/ticks/desired_position/enabled/operational/error/status.
      Effort gear-scaled (real torque, unlike Dynamixel mA passthrough).
      0x6009 errorCode PDO-mapped as INTEGER32 (3 TPDOs vs Dynamixel 2).
- [ ] Compile in the kinisi_ros dev env (no ROS on this machine) + review the
      agent-flagged items: identity check on by default; no auto-reboot-on-
      fault policy; DynamixelControl has a pre-existing dangling-reference bug
      (binds transform refs to a by-value ctor param) that MoteusControl avoids

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
