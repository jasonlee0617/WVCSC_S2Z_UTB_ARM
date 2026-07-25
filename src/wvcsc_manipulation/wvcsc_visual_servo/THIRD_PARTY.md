# Third-party-derived code

The controller helpers under `wvcsc_visual_servo/servo` were adapted from the
MIT-licensed local
`fairino_robotarm/src/visual_servo` package. Fairino-specific kinematics,
joint names, controllers, depth processing and non-PID controllers were not
copied. The MoveIt Servo process itself is provided by the installed ROS 2
package and is not vendored here.
