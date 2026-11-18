#!/usr/bin/env bash
# 复用长驻仿真容器，桌面认证和开发文件从工作空间挂载读取。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p .docker-runtime

viewer_enabled=${DOG_SIM_VIEWER:-true}
exec_env=(-e "DOG_SIM_VIEWER=$viewer_enabled")
if [[ $viewer_enabled == true ]]; then
    : "${DISPLAY:?Start this command from an X11 or XWayland desktop terminal}"
    umask 077
    runtime_dir=.docker-runtime/x11
    mkdir -p "$runtime_dir"
    auth_key=$(printf '%s\n' "$DISPLAY" "${COMPOSE_PROJECT_NAME:-}" | sha256sum)
    auth_rel="$runtime_dir/${auth_key%% *}"
    export DOG_XAUTHORITY="$PWD/$auth_rel"
    auth_tmp=$(mktemp "$runtime_dir/xauth.XXXXXX")
    trap 'unlink "$auth_tmp"' EXIT
    # 容器主机名可以与桌面不同，使用同一会话的通配认证。
    xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$auth_tmp" nmerge -
    if [[ ! -s "$auth_tmp" ]]; then
        printf 'No X11 authorization found for DISPLAY=%s\n' "$DISPLAY" >&2
        exit 1
    fi
    if ! cmp -s "$auth_tmp" "$DOG_XAUTHORITY"; then
        cat "$auth_tmp" > "$DOG_XAUTHORITY"
    fi
    unlink "$auth_tmp"
    trap - EXIT
    exec_env+=(-e "DISPLAY=$DISPLAY" -e "XAUTHORITY=/workspace/$auth_rel" -e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1)
elif [[ ! -d /tmp/.X11-unix ]]; then
    export DOG_X11_SOCKET_DIR=./docker
fi

container=$(docker compose ps --all -q sim)
if [[ -n $container ]]; then
    docker compose start --wait sim
else
    if ! docker image inspect dog-ros:sim >/dev/null 2>&1; then
        docker compose build sim
    fi
    docker compose up -d --no-recreate --no-build --pull never --wait sim
    container=$(docker compose ps -q sim)
fi

build_root=/opt/dog
if [[ $(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/opt/dog"}}mounted{{end}}{{end}}' "$container") != mounted ]]; then
    # 旧容器已有源码挂载，用它来保存产物并隔离原来自动启动的节点。
    build_root=/workspace/.docker-runtime/colcon
    exec_env+=(-e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-73}")
elif [[ -n ${ROS_DOMAIN_ID:-} ]]; then
    exec_env+=(-e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID")
fi
exec_env+=(-e "DOG_BUILD_ROOT=$build_root" -e "DOG_INSTALL=$build_root/install")

if (( $# == 0 )); then
    set -- ros2 run dog_sim simulate
fi
if [[ $1 != --build ]] && ! docker compose exec -T sim test -f "$build_root/install/setup.bash"; then
    set -- --build "$@"
fi

exec docker compose exec "${exec_env[@]}" sim bash /workspace/docker/entrypoint.sh "$@"
