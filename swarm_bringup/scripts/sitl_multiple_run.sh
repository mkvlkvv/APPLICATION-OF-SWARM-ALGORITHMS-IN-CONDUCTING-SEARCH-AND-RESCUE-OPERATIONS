#!/bin/bash
# Multi-PX4 SITL с Gazebo Classic. Все дроны инклюдятся в мир.
# Уникализируются: MAVLink-порты, имя плагина камеры, namespace и camera_name.

set -e

NUM_DRONES=${1:-5}
BASE_WORLD=${2:-"$HOME/swarm_ws_v1/src/swarm_bringup/worlds/swarm_arena.world"}
MODEL=${3:-"iris_down_cam"}

PX4_SRC="$HOME/PX4-Autopilot"
BUILD_DIR="$PX4_SRC/build/px4_sitl_default"
SRC_MODEL_DIR="$PX4_SRC/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/$MODEL"

[ -x "$BUILD_DIR/bin/px4" ]           || { echo "[ERR] PX4 not built"; exit 1; }
[ -f "$SRC_MODEL_DIR/${MODEL}.sdf" ]  || { echo "[ERR] No SDF: $SRC_MODEL_DIR/${MODEL}.sdf"; exit 1; }

# Чистка
pkill -9 -x px4        2>/dev/null || true
pkill -9 -f gzserver   2>/dev/null || true
pkill -9 -f gzclient   2>/dev/null || true
rm -rf /tmp/${MODEL}_[0-9]* /tmp/swarm_arena_runtime.world
rm -rf "$BUILD_DIR"/instance_*
sleep 2

# Окружение
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
source "$PX4_SRC/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_SRC" "$BUILD_DIR" > /dev/null

SPAWN_X=(-8 -4 0 4 8)
SPAWN_Y=(-24 -24 -24 -24 -24)

RUNTIME_WORLD="/tmp/swarm_arena_runtime.world"
sed '/<\/world>/d; /<\/sdf>/d' "$BASE_WORLD" > "$RUNTIME_WORLD"

for ((i=1; i<=NUM_DRONES; i++)); do
  idx=$((i-1))
  X=${SPAWN_X[$idx]}; Y=${SPAWN_Y[$idx]}
  TCP_PORT=$((4560 + i - 1))
  UDP_PORT=$((14560 + i - 1))

  INST_DIR="/tmp/${MODEL}_$i"
  cp -r "$SRC_MODEL_DIR" "$INST_DIR"
  SDF="$INST_DIR/${MODEL}.sdf"

  # (1) Уникальные MAVLink-порты
  sed -i "s|<mavlink_tcp_port>4560</mavlink_tcp_port>|<mavlink_tcp_port>$TCP_PORT</mavlink_tcp_port>|g" "$SDF"
  sed -i "s|<mavlink_udp_port>14560</mavlink_udp_port>|<mavlink_udp_port>$UDP_PORT</mavlink_udp_port>|g" "$SDF"

  # (2) Уникализация ROS2 camera plugin
  sed -i "s|name=\"camera_controller\"|name=\"camera_controller_$i\"|g" "$SDF"
  sed -i "s|<namespace>camera</namespace>|<namespace>camera_$i</namespace>|g" "$SDF"
  sed -i "s|<camera_name>downward</camera_name>|<camera_name>downward_$i</camera_name>|g" "$SDF"

  # (3) Переименование файла + model.config — для URI model://iris_down_cam_$i
  mv "$SDF" "$INST_DIR/${MODEL}_$i.sdf"
  if [ -f "$INST_DIR/model.config" ]; then
    sed -i "s|${MODEL}.sdf|${MODEL}_$i.sdf|g" "$INST_DIR/model.config"
    sed -i "0,/<name>.*<\/name>/s|<name>.*</name>|<name>${MODEL}_$i</name>|" "$INST_DIR/model.config"
  fi

  export GAZEBO_MODEL_PATH="/tmp:$GAZEBO_MODEL_PATH"

  cat >> "$RUNTIME_WORLD" << EOF
    <include>
      <name>${MODEL}_$i</name>
      <uri>model://${MODEL}_$i</uri>
      <pose>$X $Y 0.1 0 0 0</pose>
    </include>
EOF
done

echo "  </world>" >> "$RUNTIME_WORLD"
echo "</sdf>"     >> "$RUNTIME_WORLD"

echo "[INFO] Runtime world: $RUNTIME_WORLD"
echo "[INFO] Starting gzserver..."

gzserver --verbose "$RUNTIME_WORLD" > /tmp/gzserver.log 2>&1 &
GZ_PID=$!

for k in {1..30}; do
  if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "[ERR] gzserver died after ${k}s. Tail log:"
    tail -40 /tmp/gzserver.log
    exit 1
  fi
  if gz topic -l 2>/dev/null | grep -q "/gazebo"; then
    echo "[INFO] gzserver ready after ${k}s (pid=$GZ_PID)"
    break
  fi
  sleep 1
done

sleep 3

for ((i=1; i<=NUM_DRONES; i++)); do
  INSTANCE_DIR="$BUILD_DIR/instance_$i"
  mkdir -p "$INSTANCE_DIR"

  export PX4_SIM_MODEL="gazebo-classic_$MODEL"
  export PX4_UXRCE_DDS_NS="px4_$i"

  pushd "$INSTANCE_DIR" > /dev/null
  "$BUILD_DIR/bin/px4" -i $((i-1)) -d "$PX4_SRC/ROMFS/px4fmu_common" \
    -s "etc/init.d-posix/rcS" \
    -w "sitl_${MODEL}_$i" \
    > "$INSTANCE_DIR/px4.log" 2>&1 &
  PX4_PID=$!
  popd > /dev/null

  echo "[INFO] PX4 $i: ns=px4_$i udp=$((14560+i-1)) pid=$PX4_PID"
  sleep 2
done

echo ""
echo "[INFO] All $NUM_DRONES drones launched."
echo "[INFO] gzserver log: /tmp/gzserver.log"
echo "[INFO] PX4 logs:     $BUILD_DIR/instance_*/px4.log"
echo "[INFO] Stop:         pkill -9 -x px4; pkill -9 -f gzserver"
exit 0