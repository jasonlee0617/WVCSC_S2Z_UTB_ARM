#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo: sudo ./udev.sh" >&2
  exit 1
fi

# The deployed FDI AHRS is CH9102 (1a86:55d4).  On the industrial PC it is
# exposed by the kernel as ttyACM*, so bind the driver to a stable alias rather
# than a boot-order-dependent ttyACM number.
rule_path=/etc/udev/rules.d/99-fdilink-imu.rules
printf '%s\n' \
  'SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", GROUP="dialout", MODE="0660", SYMLINK+="FDI_IMU_GNSS"' \
  > "${rule_path}"

udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty

echo "Installed ${rule_path}; verify with: ls -l /dev/FDI_IMU_GNSS"

