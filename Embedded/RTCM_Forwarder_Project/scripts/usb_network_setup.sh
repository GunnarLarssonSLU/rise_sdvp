#!/bin/bash

IFACE="usb0"
IP="192.168.7.1"

# Bring up the USB network interface
sudo ip link set $IFACE up
sudo ip addr add $IP/24 dev $IFACE

echo "USB network interface $IFACE set to $IP"

# Optional IP forwarding if you want to route
# sudo sysctl -w net.ipv4.ip_forward=1
