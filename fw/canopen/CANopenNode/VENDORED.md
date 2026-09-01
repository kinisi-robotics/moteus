# Vendored CANopenNode

Vendored copy of CANopenNode v4, Apache License 2.0 (see LICENSE).

- Upstream: https://github.com/CANopenNode/CANopenNode
- Commit: 6dfd4ed7f22ae4b23097cf76c66a909ace5f3622 (v4.0-389-g6dfd4ed)
- This is the same commit used as a submodule by the kinisi_embedded
  gripper firmware, so protocol behavior matches the other Kinisi
  CANopen nodes.
- Omitted from the copy: example/, doc/, .github/, build files.
- Local modifications: none.  Keep it pristine; put target-specific
  code in fw/canopen/ instead (CO_driver_target.h, the FDCan driver
  bridge, and the object dictionary).
