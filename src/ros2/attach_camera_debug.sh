#!/usr/bin/env bash
# attach_camera_debug.sh
./launch_dynamic.sh dual_camera_config.yaml &
# wait until our node is running
while ! pgrep -f 'component_container_mt.*Kernel_Camera1' >/dev/null; do
  sleep 1
done
PID=$(pgrep -f 'component_container_mt.*Kernel_Camera1')
exec gdb -q -ex "attach $PID" -ex "continue"