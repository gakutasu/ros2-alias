#!/usr/bin/sh

# ROS2 default workspace path
ROS_WS=~/ros
export _colcon_cd_root=$ROS_WS
source $ROS_WS/install/setup.bash

# Find ROS workspace root
function find_ros_workspace_root() {
    local dir=$(pwd)

    while [ "$dir" != "/" ]; do
        if [ -d "$dir/src" ]; then
            echo "$dir"  # 最初に見つかった `src/` を持つディレクトリを出力
            return 0
        fi
        dir=$(dirname "$dir")
    done

    # 赤色のエラーメッセージを出力
    echo -e "\e[31mCurrent directory is not a ROS2 workspace.\e[0m" >&2
    return 1
}

function colcon_source() {
    local ws
    ws=$(find_ros_workspace_root) || return 1

    echo "Sourcing ROS2 workspace in $ws"
    source "$ws/install/setup.bash"
}

function colcon_bt() {
    local ws=$(find_ros_workspace_root) || return 1

    if [ ! -f "package.xml" ]; then
        echo "Error: package.xml not found."
        return 1
    fi

    local PKG_NAME=$(grep "<name>" package.xml | sed -e "s/<[^>]*>//g" | xargs)
    (cd "$ws" && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --parallel-workers $(nproc) --packages-up-to "$PKG_NAME" && source "$ws/install/setup.bash")
}

function colcon_clean() {
    local ws=$(find_ros_workspace_root) || return 1

    rm -rf "$ws/install" "$ws/log" "$ws/build"
}

function colcon_build() {
    local ws=$(find_ros_workspace_root) || return 1

    (cd "$ws" && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ --parallel-workers $(nproc) && source "$ws/install/setup.bash")
}

function rosdep_install() {
    local ws=$(find_ros_workspace_root) || return 1

    (cd "$ws" && rosdep install --from-paths src --ignore-src -ry)
}

alias killros='ps aux | grep ros | grep -v grep | awk '"'"'{ print "kill -9", $2 }'"'"' | sh'

source /usr/share/colcon_cd/function/colcon_cd.sh

if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
elif [ -f /usr/share/bash-completion/completions/colcon ]; then
    source /usr/share/bash-completion/completions/colcon
else
    echo "colcon completion script not found."
fi
