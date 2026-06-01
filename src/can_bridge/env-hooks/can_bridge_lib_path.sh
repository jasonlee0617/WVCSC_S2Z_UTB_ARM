#!/bin/sh

# This script is sourced by the ROS 2 environment setup script
# to add the library path to the LD_LIBRARY_PATH environment variable.

# Get the package install directory
PACKAGE_INSTALL_DIR=$(ros2 pkg prefix can_bridge)
echo $PACKAGE_INSTALL_DIR
# Add the library directory to the LD_LIBRARY_PATH
export LD_LIBRARY_PATH="$PACKAGE_INSTALL_DIR/lib/can_bridge:$LD_LIBRARY_PATH"   