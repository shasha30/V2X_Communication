#!/bin/bash
set -e
DISPLAY=":1"
VNC_PORT=5901
NOVNC_PORT=6080
SUMO_CFG="corridor.sumocfg"   # change if needed
SUMO_BIN="$(which sumo-gui || echo /usr/bin/sumo-gui)"

echo "Starting Xvfb on $DISPLAY..."
Xvfb $DISPLAY -screen 0 1280x720x24 -ac >/tmp/xvfb.log 2>&1 &
sleep 0.6

echo "Starting x11vnc on $DISPLAY (port $VNC_PORT)..."
x11vnc -display $DISPLAY -nopw -forever -shared -rfbport $VNC_PORT >/tmp/x11vnc.log 2>&1 &
sleep 0.6

echo "Starting websockify (noVNC) on port $NOVNC_PORT -> $VNC_PORT..."
# try to use system websockify; if not, try python3 -m websockify
if command -v websockify >/dev/null 2>&1; then
  websockify $NOVNC_PORT localhost:$VNC_PORT --web /usr/share/novnc/ >/tmp/websockify.log 2>&1 &
else
  python3 -m websockify $NOVNC_PORT localhost:$VNC_PORT --web /usr/share/novnc/ >/tmp/websockify.log 2>&1 &
fi
sleep 0.6

echo "Launching sumo-gui on $DISPLAY (config: $SUMO_CFG)..."
export DISPLAY=$DISPLAY
$SUMO_BIN -c $SUMO_CFG >/tmp/sumo-gui.log 2>&1 &
sleep 1

echo "Started. Check logs: /tmp/xvfb.log /tmp/x11vnc.log /tmp/websockify.log /tmp/sumo-gui.log"
echo "Open in browser: http://$(hostname -I | awk '{print $1}'):$NOVNC_PORT/vnc.html"
