# Calibration dependencies

The `easy_handeye2` and `ros2_aruco` trees are the generic calibration
implementations migrated from
`/home/robot/fairino_robotarm/src/calibration_stack` at source commit
`3e06d1f8af6992ee97b6a9fc009b0beaa0965a96`.

They are intentionally kept robot-independent. WVCSC-specific frames, marker
contract and export validation live in `wvcsc_calibration`; no Fairino planner,
driver or frame name is used by the deployment wrapper. Upstream license files
are retained beside each imported tree.
