#!/usr/bin/env bash
set -u

SCRIPTS_DIR="$HOME/Documents/ros2_ws"
CHECK_TOPIC="/mavros/state"

: <<'COMMENT'
The auto_environment.sh file is supposed to launch 2 terminals, run the 2 "source" lines 
source /opt/ros/jazzy/setup.bash adds ROS and mavros.
source ~/Documents/ros2_ws/install/setup.bash adds your own packages (idt).
and allow the user to decide what launch command to use depending on the hardware setup.
to make it even quicker to run this, you can make a .desktop file with the snippet below adapted to your auto_environment

[Desktop Entry]
Version=1.0
Type=Application
Name=MAVROS Launcher
Comment=Start MAVROS node and connection check tab
Exec=/home/alessandro/launch-mavros.sh          #launch-mavros.sh needs to be the name of the laucher file
Path=/home/alessandro/Documents/ros2_ws         #path that the launcher will set the terminal to
Terminal=false
Categories=Development;
#the path on the Exec line must be set correctly ofc.
COMMENT

# Tab 1: MAVROS Check (creates the window, maximized but not fullscreen)
gnome-terminal \
  --window \
  --maximize \
  --title="MAVROS Check" \
  -- bash -c '
    SCRIPTS_DIR="$1"
    CHECK_TOPIC="$2"

    source /opt/ros/jazzy/setup.bash
    source "$SCRIPTS_DIR/install/setup.bash"
    cd "$SCRIPTS_DIR" || exit 1

    sleep 8

    echo "Checking $CHECK_TOPIC for 4 seconds..."
    timeout -k 2 4 ros2 topic echo --once "$CHECK_TOPIC" || true

    echo "Check finished. Shell ready in $(pwd)."
    exec bash
  ' _ "$SCRIPTS_DIR" "$CHECK_TOPIC"

# Give the terminal server a moment to register the window
sleep 1

# Tab 2: mavROS node (added to the same window, ends up focused)
gnome-terminal \
  --tab \
  --title="mavROS node" \
  -- bash -c '
    SCRIPTS_DIR="$1"

    source /opt/ros/jazzy/setup.bash
    source "$SCRIPTS_DIR/install/setup.bash"

    echo "Select MAVROS mode:"
    echo "  1) Radio modem connected + UDP forwarding"
    echo "  2) PX4 wired connection + UDP forwarding"
    echo "  3) Radio modem connected, NO UDP forwarding"

    read -r -p "Enter 1, 2, or 3: " choice

    case "$choice" in
      1)
        MAVROS_CMD="ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@192.168.152.1:14550"
        ;;
      2)
        MAVROS_CMD="ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@192.168.152.1:14550"
        ;;
      3)
        MAVROS_CMD="ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyUSB0:57600"
        ;;
      *)
        echo "Invalid selection. Exiting."
        sleep 3
        exit 1
        ;;
    esac

    echo "Launching: $MAVROS_CMD"
    eval "$MAVROS_CMD"

    exec bash
  ' _ "$SCRIPTS_DIR"