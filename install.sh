#!/bin/bash
# Lionel MTH Bridge Installation Script
# Automatically installs dependencies and sets up the bridge

set -e  # Exit on any error

echo "🚂 Lionel MTH Bridge Installation Script"
echo "========================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_warning "Running as root. This may not be necessary for all operations."
fi

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    if command -v apt-get &> /dev/null; then
        DISTRO="debian"
    elif command -v yum &> /dev/null; then
        DISTRO="redhat"
    elif command -v pacman &> /dev/null; then
        DISTRO="arch"
    else
        print_error "Unsupported Linux distribution"
        exit 1
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
else
    print_error "Unsupported operating system: $OSTYPE"
    exit 1
fi

print_status "Detected OS: $OS"
if [ "$OS" = "linux" ]; then
    print_status "Detected distribution: $DISTRO"
fi

# Create virtual environment first (required for PEP 668 compliant systems)
print_status "Creating Python virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install required Python packages in venv
print_status "Installing required Python packages..."
pip install pyserial zeroconf

# Create configuration directory
print_status "Creating configuration directory..."
CONFIG_DIR="$HOME/.lionel-mth-bridge"
mkdir -p "$CONFIG_DIR"

# Copy default configuration if it doesn't exist
if [ ! -f "$CONFIG_DIR/bridge_config.json" ]; then
    print_status "Creating default configuration..."
    cp bridge_config.json "$CONFIG_DIR/bridge_config.json"
else
    print_warning "Configuration already exists at $CONFIG_DIR/bridge_config.json"
fi

# Create systemd service file
print_status "Creating systemd service..."
SERVICE_FILE="/etc/systemd/system/lionel-mth-bridge.service"

# Create service file content
SERVICE_CONTENT="[Unit]
Description=Lionel MTH Bridge Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/venv/bin/python $(pwd)/lionel_mth_bridge.py
Restart=always
RestartSec=10
Environment=PYTHONPATH=$(pwd)

[Install]
WantedBy=multi-user.target"

# Use sudo to write the service file
echo "$SERVICE_CONTENT" | sudo tee "$SERVICE_FILE" > /dev/null

# Reload systemd and enable service
sudo systemctl daemon-reload
sudo systemctl enable lionel-mth-bridge.service
print_status "Systemd service created and enabled"

# Create startup scripts
print_status "Creating startup scripts..."

# Main startup script
cat > start_bridge.sh << 'EOF'
#!/bin/bash
# Start Lionel MTH Bridge

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Change to script directory
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies if needed
pip install -r requirements.txt 2>/dev/null || true

# Start the bridge
echo "🚂 Starting Lionel MTH Bridge..."
python3 lionel_mth_bridge.py
EOF

# Make scripts executable
chmod +x start_bridge.sh

# Create requirements.txt
print_status "Creating requirements.txt..."
cat > requirements.txt << 'EOF'
pyserial>=3.5
zeroconf>=0.39.0
EOF

# Create log directory
print_status "Creating log directory..."
mkdir -p logs

# Test installation
print_status "Testing installation..."
source venv/bin/activate

# Test Python imports
python3 -c "
import serial
import socket
import json
import threading
import time
print('✅ All required modules imported successfully')
"

if [ $? -eq 0 ]; then
    print_status "✅ Installation completed successfully!"
else
    print_error "❌ Installation test failed"
    exit 1
fi

# Print next steps
echo ""
echo "🎉 Installation Complete!"
echo "========================"
echo ""
echo "Power Cycle the Arduino Uno Q"
