#!/bin/bash
set -e  # Exit on error

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Use 'sudo ./setup.sh'."
    exit 1
fi

# Define variables
APP_NAME="rise_sdvp"
APP_DIR="/opt/$APP_NAME"
SERVICE_NAME="$APP_NAME.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"  # Directory where this script is located

# Install dependencies
echo "Installing system dependencies..."
apt-get update -y
apt-get install -y python3 python3-pip python3-venv

# Create app directory
echo "Creating app directory at $APP_DIR..."
mkdir -p "$APP_DIR"

# Copy all necessary files to app directory
echo "Copying files to $APP_DIR..."
cp "$SCRIPT_DIR/server.py" "$APP_DIR/"
cp "$SCRIPT_DIR/streamlit_app.py" "$APP_DIR/"
cp "$SCRIPT_DIR/requirements.txt" "$APP_DIR/"

# Copy database and XML files if they exist
if [ -f "$SCRIPT_DIR/data.db" ]; then
    cp "$SCRIPT_DIR/data.db" "$APP_DIR/"
fi
cp "$SCRIPT_DIR/fields.xml" "$APP_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/locations.xml" "$APP_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/machines.xml" "$APP_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/paths.xml" "$APP_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/vehicle_types.xml" "$APP_DIR/" 2>/dev/null || true
cp "$SCRIPT_DIR/found_machines.xml" "$APP_DIR/" 2>/dev/null || true

# Copy the entire fields directory if it exists
if [ -d "$SCRIPT_DIR/fields" ]; then
    cp -r "$SCRIPT_DIR/fields" "$APP_DIR/"
fi

# Create a virtual environment
echo "Creating Python virtual environment..."
python3 -m venv "$APP_DIR/venv"

# Install Python dependencies
echo "Installing Python dependencies..."
"$APP_DIR/venv/bin/pip" install -r "$APP_DIR/requirements.txt"

# Create systemd service file for server.py
echo "Creating systemd service for $APP_NAME..."
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=$APP_NAME Server
After=network.target

[Service]
User=root
WorkingDirectory=$APP_DIR
Environment="PATH=$APP_DIR/venv/bin:$PATH"
ExecStart=$APP_DIR/venv/bin/python $APP_DIR/server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable the service
echo "Reloading systemd and enabling $SERVICE_NAME..."
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl start "$SERVICE_NAME"

# Check if the service is running
echo "Checking if $SERVICE_NAME is running..."
systemctl status "$SERVICE_NAME" --no-pager

echo ""
echo "========================================="
echo "Setup complete!"
echo "========================================="
echo "Your server is now running as a systemd service and will start automatically at boot."
echo ""
echo "To check the service status, run:"
echo "  sudo systemctl status $SERVICE_NAME"
echo ""
echo "To view logs, run:"
echo "  sudo journalctl -u $SERVICE_NAME -f"
echo ""
echo "To start the Streamlit interface, run:"
echo "  cd $APP_DIR && $APP_DIR/venv/bin/streamlit run streamlit_app.py"
echo ""
echo "Note: You may need to open the Streamlit port (default: 8501) in your firewall if accessing remotely."
