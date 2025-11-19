#!/bin/bash
# Yahboom Rosmaster R2 - Power Switch Location Guide

cat << 'EOF'

╔════════════════════════════════════════════════════════════════╗
║     YAHBOOM ROSMASTER R2 - POWER SWITCH LOCATIONS              ║
╚════════════════════════════════════════════════════════════════╝

The battery power switch can be in several locations:

┌────────────────────────────────────────────────────────────────┐
│ LOCATION 1: Side Panel (Most Common)                          │
│ • Look on the RIGHT side of the robot                         │
│ • Near the battery compartment                                │
│ • Red rocker switch labeled "POWER" or battery symbol         │
│ • Red LED next to it (lights up when ON)                      │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│ LOCATION 2: Back Panel                                        │
│ • Behind the robot near USB ports                             │
│ • May be integrated with emergency stop button                │
│ • Look for red switch or button                               │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│ LOCATION 3: Under Top Deck                                    │
│ • Remove 4 screws from top acrylic deck                       │
│ • Switch may be mounted on the motor controller board         │
│ • Usually near the battery connector                          │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│ WHAT TO LOOK FOR:                                             │
│                                                                │
│   [●] ⚡ POWER    ← Red rocker switch                        │
│    ↑                                                          │
│   LED (should be RED when ON)                                 │
│                                                                │
│ OR                                                             │
│                                                                │
│   ( O )  ← Round red emergency stop button                   │
│            (twist to release if pressed)                      │
└────────────────────────────────────────────────────────────────┘

╔════════════════════════════════════════════════════════════════╗
║ AFTER FINDING THE SWITCH:                                     ║
╚════════════════════════════════════════════════════════════════╝

1. Toggle switch to ON position
2. Check for RED LED to light up
3. Run: python3 check_power.py
4. Voltage should show > 11V

╔════════════════════════════════════════════════════════════════╗
║ IF NO LED LIGHTS UP:                                          ║
╚════════════════════════════════════════════════════════════════╝

The battery may be:
  • Completely discharged (0%)
  • Disconnected from motor board
  • Faulty or damaged

Next steps:
  1. Open chassis (remove top deck screws)
  2. Check battery connector to motor board
  3. Check battery voltage with multimeter (should be ~12V)
  4. May need to charge battery separately

╔════════════════════════════════════════════════════════════════╗
║ BATTERY CHARGING NOTES:                                       ║
╚════════════════════════════════════════════════════════════════╝

⚠️  IMPORTANT: Wall charger powers Jetson ONLY, not motors!

To charge the battery:
  • Battery must be connected AND switch ON
  • OR remove battery and charge with dedicated charger
  • Charging time: 2-4 hours (depending on depletion)
  • Full charge: 12.6V (3S LiPo) or 12.0V (3S Li-ion)

EOF

read -p "Press Enter to continue..."

echo ""
echo "Would you like to:"
echo "  1) Check power now (run power diagnostic)"
echo "  2) Exit"
echo ""
read -p "Choice (1 or 2): " choice

if [ "$choice" = "1" ]; then
    python3 "$(dirname "$0")/check_power.py"
fi
