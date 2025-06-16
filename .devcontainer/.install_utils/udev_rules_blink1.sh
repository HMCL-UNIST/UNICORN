#!/bin/bash

# Script to install udev rules for blinkmk3, IMU, and VESC devices.

set -e

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "❌ Please run this script as root (e.g., with sudo)"
  exit 1
fi

echo "🔧 Creating udev rules in /etc/udev/rules.d/"

# Create 99-blinkmk3.rules
cat <<EOF > /etc/udev/rules.d/99-blinkmk3.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="27b8", ATTRS{idProduct}=="01ed", MODE="0666", GROUP="plugdev"
EOF
echo "✅ Created 99-blinkmk3.rules"

# Reload and apply the new rules
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "🎉 Blink1 udev rules installed and applied successfully!"

