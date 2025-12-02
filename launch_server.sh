#!/bin/bash

echo "=== Cleaning previous processes ==="

# Kill previous camera scripts
pkill -9 -f robot_camera.py 2>/dev/null

# Kill any python process listening on camera ports
sudo fuser -k 10005/tcp 2>/dev/null
sudo fuser -k 11005/tcp 2>/dev/null

# Kill server port 5000
sudo fuser -k 5000/tcp 2>/dev/null

echo "=== Previous processes cleared ==="

# Prepare logs directory
mkdir -p logs

echo "=== Starting robot_camera.py ==="
nohup python3 robot_camera.py > logs/camera_log.txt 2>&1 &

cd server

echo "=== Starting gunicorn server ==="
nohup gunicorn -w 12 -b 0.0.0.0:5000 -k gevent --timeout 0 \
  --worker-connections 2 "monitor:app" > ../logs/cam_server.txt 2>&1 &

echo "=== All services started successfully ==="