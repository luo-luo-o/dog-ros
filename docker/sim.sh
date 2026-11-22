#!/usr/bin/env bash
# 复用长驻仿真容器，桌面认证和开发文件从工作空间挂载读取。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p .docker-runtime

viewer_enabled=${DOG_SIM_VIEWER:-true}
exec_env=(-e "DOG_SIM_VIEWER=$viewer_enabled")
wslg_socket_dir=/mnt/wslg/.X11-unix
wslg_socket=
if [[ ${DISPLAY:-} =~ ^:([0-9]+)(\.[0-9]+)?$ ]] &&
    [[ -S $wslg_socket_dir/X${BASH_REMATCH[1]} ]]; then
    wslg_socket=/tmp/.X11-unix/X${BASH_REMATCH[1]}
fi
# WSLg 可用时，无窗口和开窗启动共用同一个套接字挂载。
if [[ -n $wslg_socket || ( $viewer_enabled != true && -d $wslg_socket_dir ) ]]; then
    export DOG_X11_SOCKET_DIR=${DOG_X11_SOCKET_DIR:-$wslg_socket_dir}
elif [[ $viewer_enabled != true && ! -d /tmp/.X11-unix ]]; then
    export DOG_X11_SOCKET_DIR=${DOG_X11_SOCKET_DIR:-./docker}
fi

if [[ $viewer_enabled == true ]]; then
    : "${DISPLAY:?Start from an X11, XWayland or WSLg terminal, or set DOG_SIM_VIEWER=false}"
    exec_env+=(-e "DISPLAY=$DISPLAY" -e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1)
fi
if [[ $viewer_enabled == true && -n $wslg_socket ]]; then
    # WSLg 的 XWayland 默认通过本地套接字接入，无需桌面会话的认证文件。
    export DOG_XAUTHORITY=/dev/null
    exec_env+=(-e XAUTHORITY=/dev/null)
elif [[ $viewer_enabled == true ]]; then
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
    exec_env+=(-e "XAUTHORITY=/workspace/$auth_rel")
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

if [[ $viewer_enabled == true && -n $wslg_socket ]] &&
    ! docker compose exec -T sim test -S "$wslg_socket"; then
    # 此前以无桌面模式创建的容器可能挂载了占位目录，应用新的挂载配置。
    docker compose up -d --no-build --pull never --wait sim
    container=$(docker compose ps -q sim)
    if ! docker compose exec -T sim test -S "$wslg_socket"; then
        printf 'WSLg X11 socket %s is missing in sim. Check the Docker bind mount from %s.\n' \
            "$wslg_socket" "$DOG_X11_SOCKET_DIR" >&2
        exit 1
    fi
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
