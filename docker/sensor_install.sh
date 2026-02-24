echo "=========================================="
echo "Tactile Sensor Insatll"
echo "=========================================="
echo ""
echo "Gives permissions and creates symlink used in ros pacakges"

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="$SCRIPT_DIR/.venvSimpleCheck"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color


# Load helper scripts from parent directory
echo "Loading helper scripts..."
if [ -f "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/apply_udev_rule.sh" ]; then
    source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/apply_udev_rule.sh"
    echo -e "${GREEN}✓ Loaded apply_udev_rule.sh${NC}"
else
    echo -e "${YELLOW}Warning: apply_udev_rule.sh not found, skipping...${NC}"
    apply_udev_rule() { :; }  # No-op function
fi

if [ -f "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/set_sensor_permissions.sh" ]; then
    source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/set_sensor_permissions.sh"
    echo -e "${GREEN}✓ Loaded set_sensor_permissions.sh${NC}"
else
    echo -e "${YELLOW}Warning: set_sensor_permissions.sh not found, skipping...${NC}"
    set_sensor_permissions() { :; }  # No-op function
fi

if [ -f "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/find_sensor_devices.sh" ]; then
    source "${PROJECT_ROOT}/extern/tactile_sensors/utils/scripts/find_sensor_devices.sh"
    echo -e "${GREEN}✓ Loaded find_sensor_devices.sh${NC}"
else
    echo -e "${YELLOW}Warning: find_sensor_devices.sh not found, skipping...${NC}"
    find_sensor_devices() { echo ""; }  # Return empty
fi

echo ""
echo "=========================================="
echo "Configuring Sensor Permissions"
echo "=========================================="

# Step 4: Apply udev rules -> handled by udev rules?
echo ""
echo "[1/3] Applying udev rules..."
apply_udev_rule

# # Step 5: Set sensor permissions
echo ""
echo "[2/3] Setting sensor permissions..."
set_sensor_permissions

# Step 6: Find sensor devices
echo ""
echo "[3/3] Finding sensor devices..."
sensor_devices=($(find_sensor_devices))

if ((${#sensor_devices[@]} == 0)); then
    echo ""
    echo -e "${YELLOW}=========================================="
    echo "Warning: No sensor devices detected"
    echo "==========================================${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure the sensor is plugged in"
    echo "2. Try unplugging and replugging the sensor"
    echo "3. Check if you're in the dialout group: groups"
    echo "4. You may need to log out and back in"
    echo ""
    read -p "Continue anyway? (y/N): " response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
else
    echo -e "${GREEN}✓ Found ${#sensor_devices[@]} sensor device(s):${NC}"
    for dev in "${sensor_devices[@]}"; do
        echo "  - $dev"
    done
fi
