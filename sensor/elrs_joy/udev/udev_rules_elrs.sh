#!/bin/bash

# Script to install udev rules for ELRS receiver.
# Two USB-TTL chip families supported, mapped to distinct symlinks so they
# coexist on the same host and the launch file selects one explicitly:
#   CP2102  (Silicon Labs, 10c4:ea60) -> /dev/ELRS         (legacy adapter)
#   FT232RL (FTDI,         0403:6001) -> /dev/ELRS_FT232   (preferred, hits 420000 baud exactly)

set -e

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run this script as root (e.g., with sudo)"
  exit 1
fi

echo "Creating udev rules in /etc/udev/rules.d/"

# Create 99-elrs.rules
cat <<EOF > /etc/udev/rules.d/99-elrs.rules
# CP2102 (Silicon Labs) - legacy ELRS USB-TTL, capped at 115200 in practice
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ELRS", MODE="0666"
# FT232RL (FTDI) - preferred adapter, can hit 420000 baud exactly for full CRSF + CRC
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ELRS_FT232", MODE="0666"
# latency_timer=1: FT232R defaults to 16ms USB read-batching, which adds up to
# ~16ms input lag (felt as throttle/steering lag on fast stick swipes). Force 1ms
# so bytes reach the host promptly. NOTE: latency_timer lives on the usb-serial
# parent device, NOT the tty class device, so it must be set on SUBSYSTEM=="usb-serial"
# (a tty rule silently no-ops). Scoped to the FTDI driver via DRIVER=="ftdi_sio".
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
EOF
echo "Created 99-elrs.rules"

# Reload and apply the new rules
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "ELRS udev rules installed and applied successfully!"
echo "Symlinks: CP2102 -> /dev/ELRS, FT232RL -> /dev/ELRS_FT232"
