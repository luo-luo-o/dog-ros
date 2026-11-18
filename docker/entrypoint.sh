#!/usr/bin/env bash
# 启动时构建挂载的 ROS 包。容器内运行额外命令时复用已安装的工作空间。
set -e
source /opt/ros/humble/setup.bash

if [[ ${1:-} == --build || ${1:-} == --ensure-build ]]; then
    build_mode=$1
    shift
    build_root=${DOG_BUILD_ROOT:-/opt/dog}
    if [[ $build_root == /opt/dog ]]; then
        ready_file=/tmp/dog-ros-build-ready
        : > "$ready_file"
    fi
    if [[ $build_mode == --build || ! -f $DOG_INSTALL/setup.bash ]]; then
        colcon --log-base "$build_root/log" build \
            --base-paths /workspace/src \
            --build-base "$build_root/build" \
            --install-base "${DOG_INSTALL}" \
            --merge-install
    fi
fi

source "${DOG_INSTALL}/setup.bash"
if [[ -n ${ready_file:-} ]]; then
    printf 'ready\n' > "$ready_file"
fi
exec "$@"
