#!/bin/bash
# Запуск Gazebo GUI (gzclient) с правильным окружением под WSLg

# Источник Gazebo окружения (ОБЯЗАТЕЛЬНО для шейдеров)
source /usr/share/gazebo/setup.sh

# Источник PX4 путей к моделям (чтобы видел iris_down_cam и пр.)
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default > /dev/null

# Добавляем /tmp — там лежат iris_down_cam_1..5
export GAZEBO_MODEL_PATH="/tmp:$GAZEBO_MODEL_PATH"

# --- Настройки рендера для WSLg ---
# Ogre в Gazebo 11 иногда ломается с D3D12; подстраховка:
export OGRE_RTT_MODE=Copy         # избегаем PBuffer (часто причина сегфолта)
unset LIBGL_ALWAYS_INDIRECT        # в WSLg не нужно
# Если сегфолт сохранится — раскомментируй следующую строку для software rendering:
# export LIBGL_ALWAYS_SOFTWARE=1

echo "[INFO] GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH"
echo "[INFO] GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "[INFO] DISPLAY=$DISPLAY WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
echo "[INFO] Starting gzclient..."

exec gzclient --verbose