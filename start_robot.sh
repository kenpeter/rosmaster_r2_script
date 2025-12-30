#!/bin/bash
# DEPRECATED: This script has been consolidated into start_auto.py
# Use start_auto.py --robot-only for robot hardware only
# Or use start_auto.py for full system (robot + autonomous)

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${YELLOW}=========================================${NC}"
echo -e "${YELLOW}  ⚠️  DEPRECATION NOTICE${NC}"
echo -e "${YELLOW}=========================================${NC}"
echo ""
echo -e "${CYAN}This script (start_robot.sh) has been consolidated into start_auto.py${NC}"
echo ""
echo -e "${GREEN}New unified startup options:${NC}"
echo -e "  ${CYAN}./start_auto.py${NC}              # Full system (robot + autonomous + SLAM)"
echo -e "  ${CYAN}./start_auto.py --robot-only${NC} # Robot hardware only (what this script did)"
echo -e "  ${CYAN}./start_auto.py --no-rtabmap${NC} # Robot + autonomous (no SLAM)"
echo -e "  ${CYAN}./start_auto.py --no-fusion${NC}  # Robot + SLAM (no autonomous)"
echo ""
echo -e "${YELLOW}Benefits of consolidated script:${NC}"
echo -e "  ✅ Single command to start everything"
echo -e "  ✅ Automatic dependency management"
echo -e "  ✅ Better error handling"
echo -e "  ✅ Cleaner startup sequence"
echo ""
echo -e "${GREEN}Redirecting to: ./start_auto.py --robot-only${NC}"
echo ""

# Small delay so user can see the message
sleep 2

# Execute start_auto.py with --robot-only flag
exec "$SCRIPT_DIR/start_auto.py" --robot-only "$@"
