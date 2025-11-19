#!/bin/bash
# WiFi network switching script

echo "Disconnecting from ROSMASTER..."
sudo nmcli connection down ROSMASTER

echo "Connecting to netgear-2.4g_EXT..."
sudo nmcli device wifi connect "netgear-2.4g_EXT" password "netgear123"

echo "WiFi switch complete!"
nmcli connection show --active
