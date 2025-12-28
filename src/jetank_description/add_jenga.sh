#!/bin/bash

# bash ~/jetank_ws/src/jetank/jetank_rl/jetank_rl/add_jenga.sh

MODEL_PATH="/home/junho/jetank_ws/src/jetank_description/jenga.sdf"

WORLD="empty"
COUNT=54

# 기본 좌표
BASE_X="-0.28"
BASE_Y="0.03"
BASE_Z="0.08"    # 바닥 충돌 방지: 0.08 추천

# 랜덤 offset 범위
RANGE_X="0.10"
RANGE_Y="0.10"

# yaw 범위 (rad)
YAW_MIN="-0.8"
YAW_MAX="0.8"

# yaw → quaternion 변환
quat_from_yaw() {
    local yaw="$1"
    local half=$(echo "$yaw / 2" | bc -l)
    local qz=$(echo "s($half)" | bc -l)
    local qw=$(echo "c($half)" | bc -l)
    echo "$qz $qw"
}

# 랜덤 float
rand_float() {
    awk -v min="$1" -v max="$2" \
        'BEGIN{srand(); print min + (max-min)*rand()}'
}

echo "📦 Jenga 랜덤 spawn 시작 (총 ${COUNT}개)"

#   COUNT개 모두 SDF로 개별 스폰
for ((i=1; i<=COUNT; i++)); do

    NAME="jenga${i}"

    # 랜덤 위치
    X=$(rand_float $(echo "$BASE_X-$RANGE_X" | bc) $(echo "$BASE_X+$RANGE_X" | bc))
    Y=$(rand_float $(echo "$BASE_Y-$RANGE_Y" | bc) $(echo "$BASE_Y+$RANGE_Y" | bc))
    Z="$BASE_Z"

    # 랜덤 yaw
    YAW=$(rand_float "$YAW_MIN" "$YAW_MAX")
    read QZ QW <<< "$(quat_from_yaw $YAW)"

    echo "spawn: $NAME (yaw=$YAW)"

    ign service -s /world/${WORLD}/create \
      --reqtype ignition.msgs.EntityFactory \
      --reptype ignition.msgs.Boolean \
      --timeout 300 \
      --req "sdf_filename: \"${MODEL_PATH}\",
            name: \"${NAME}\",
            pose: {
                position: { x: ${X}, y: ${Y}, z: ${Z} },
                orientation: { x: 0, y: 0, z: ${QZ}, w: ${QW} }
            }"

    sleep 0.1
done

echo "랜덤 spawn 완료!"