#! /bin/bash

# Runs inside the container: starts a virtual display (:1) with a window manager
# and a VNC server on port 5900, so GUIs (RViz, Gazebo) can be viewed from the host
# with any VNC viewer. Software OpenGL (Mesa llvmpipe) is used for rendering.
# Safe to call more than once.

export DISPLAY=:1

if ! pgrep -x Xvfb > /dev/null; then
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
    Xvfb :1 -screen 0 1600x1000x24 +extension GLX > /tmp/xvfb.log 2>&1 &
    sleep 1
fi
# Open RViz and Gazebo windows maximized
mkdir -p ~/.fluxbox
printf '[app] (class=rviz2)\n  [Maximized] {yes}\n[end]\n[app] (class=.*[Gg]azebo.*)\n  [Maximized] {yes}\n[end]\n' > ~/.fluxbox/apps
pgrep -x fluxbox > /dev/null || (fluxbox > /tmp/fluxbox.log 2>&1 &)
pgrep -x x11vnc > /dev/null || x11vnc -display :1 -forever -shared -passwd racer -rfbport 5900 -bg -quiet -o /tmp/x11vnc.log
