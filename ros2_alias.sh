#!/usr/bin/sh

# ROS2 default workspace path
ROS_WS=~/ros
export _colcon_cd_root=$ROS_WS
source $ROS_WS/install/setup.bash

# Find workspace root
find_ros_workspace_root() {
    local dir=$(pwd)
    while [ "$dir" != "/" ]; do
        if [ -d "$dir/src" ]; then
            echo "$dir"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    echo -e "\e[31mNot a ROS2 workspace.\e[0m" >&2
    return 1
}

REAL_COLCON="$(command -v colcon)"
REAL_ROSDEP="$(command -v rosdep)"

colcon() {
    local sub="$1"
    shift
    case "$sub" in
        source)
            local ws
            ws=$(find_ros_workspace_root) || return 1
            [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash"
            ;;
        bt)
            local ws
            ws=$(find_ros_workspace_root) || return 1
            [ ! -f "package.xml" ] && echo "package.xml not found." && return 1
            local pkg_name
            pkg_name=$(grep "<name>" package.xml | sed -e "s/<[^>]*>//g" | xargs)
            (cd "$ws" && "$REAL_COLCON" build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                -DCMAKE_BUILD_TYPE=Release \
                --parallel-workers "$(nproc)" --packages-up-to "$pkg_name")
            [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash"
            ;;
        build)
            local ws
            ws=$(find_ros_workspace_root) || return 1
            if [ $# -gt 0 ]; then
                (cd "$ws" && "$REAL_COLCON" build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                    -DCMAKE_BUILD_TYPE=Release \
                    --parallel-workers "$(nproc)" --packages-up-to "$1")
            else
                (cd "$ws" && "$REAL_COLCON" build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                    -DCMAKE_BUILD_TYPE=Release \
                    --parallel-workers "$(nproc)")
            fi
            [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash"
            ;;
        clean)
            local ws
            ws=$(find_ros_workspace_root) || return 1
            rm -rf "$ws/build" "$ws/install" "$ws/log"
            ;;
        kill)
            ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
            ;;
        *)
            "$REAL_COLCON" "$sub" "$@"
            ;;
    esac
}

rosdep() {
    local sub="$1"
    shift
    case "$sub" in
        install)
            local ws
            ws=$(find_ros_workspace_root) || return 1
            (cd "$ws" && "$REAL_ROSDEP" install --from-paths src --ignore-src -ry "$@")
            ;;
        *)
            "$REAL_ROSDEP" "$sub" "$@"
            ;;
    esac
}

source /usr/share/colcon_cd/function/colcon_cd.sh

if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
elif [ -f /usr/share/bash-completion/completions/colcon ]; then
    source /usr/share/bash-completion/completions/colcon
else
    echo "No colcon completion."
fi
