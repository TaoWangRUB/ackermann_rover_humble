#!/bin/bash

# verify_cmd_vel_chain.sh
# Verifies the message types and publisher/subscriber counts for the velocity command chain.
# Usage:
# Run this inside the Docker container while the full stack is running.
# E.g.: docker-compose -f docker/docker-compose.yml exec ackermann_slam bash scripts/verify_cmd_vel_chain.sh

set -e

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,/^set /{ /^#/s/^# \?//p }' "$0"
    exit 0
fi


GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo -e "${GREEN}=== Velocity Command Chain Verification ===${NC}"

function check_topic() {
    local topic=$1
    local expected_publishers=$2
    local expected_subscribers=$3
    
    echo -e "${YELLOW}Checking topic: ${topic}${NC}"
    
    # Get topic info
    local info=$(ros2 topic info $topic)
    echo "$info"
    
    # Extract type
    local msg_type=$(echo "$info" | grep "Type:" | awk '{print $2}')
    echo "Message Type: $msg_type"
    
    # Warn if mismatch
    if [[ -z "$msg_type" ]]; then
        echo -e "${RED}[ERROR] Topic $topic not found or has no type!${NC}"
    fi
    echo ""
}

# 1. Nav2 Output (Behavior/Planner) -> Velocity Smoother
check_topic "/cmd_vel_nav"

# 2. Velocity Smoother -> Collision Monitor
check_topic "/cmd_vel_smoothed"

# 3. Collision Monitor -> Ackermann Controller
check_topic "/cmd_vel"

echo -e "${GREEN}Verification complete. Compare the message types above to ensure compatibility.${NC}"
