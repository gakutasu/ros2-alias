#!/usr/bin/sh

# ROS2 default workspace path
ROS_WS=~/ros
export _colcon_cd_root=$ROS_WS

export LIBGL_ALWAYS_SOFTWARE=1 # Disable hardware acceleration for GUI tools
export RCUTILS_COLORIZED_OUTPUT=1 # Enable colorized output
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}' # Simplify log output

# Find ROS workspace root
function find_ros_workspace_root() {
    local dir=$(pwd)
    while [ "$dir" != "/" ]; do
        if [ -d "$dir/src" ]; then
            for subdir in "$dir/src"/*/; do
                if [ -f "${subdir}package.xml" ]; then
                    echo "$dir"
                    return
                fi
            done
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

function colcon_source() {
    local ws=$(find_ros_workspace_root)
    if [ -z "$ws" ]; then
        echo "Current directory is not a ROS2 workspace."
        return 1
    fi
    echo "Sourcing ROS2 workspace in $ws"
    source "$ws/install/setup.bash"
}

function colcon_bt() {
    local ws=$(find_ros_workspace_root)
    if [ -z "$ws" ]; then
        echo "No ROS workspace found."
        return 1
    fi

    if [ ! -f "package.xml" ]; then
        echo "Error: package.xml not found."
        return 1
    fi

    local PKG_NAME=$(grep "<name>" package.xml | sed -e "s/<[^>]*>//g" | xargs)
    (cd "$ws" && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc) --packages-up-to "$PKG_NAME" && source install/setup.bash)
}

function colcon_clean() {
    local ws=$(find_ros_workspace_root)
    if [ -z "$ws" ]; then
        echo "No ROS workspace found."
        return 1
    fi
    rm -rf "$ws/install" "$ws/log" "$ws/build"
}

function colcon_build() {
    local ws=$(find_ros_workspace_root)
    if [ -z "$ws" ]; then
        echo "No ROS workspace found."
        return 1
    fi
    (cd "$ws" && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc) && source install/setup.bash)
}

function rosdep_install() {
    local ws=$(find_ros_workspace_root)
    if [ -z "$ws" ]; then
        echo "No ROS workspace found."
        return 1
    fi
    (cd "$ws" && rosdep install -r --from-paths /src --ignore-src -y)
}

alias kill_ros_processes='ps aux | grep ros | grep -v grep | awk '"'"'{ print "kill -9", $2 }'"'"' | sh'

source /usr/share/colcon_cd/function/colcon_cd.sh

if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
elif [ -f /usr/share/bash-completion/completions/colcon ]; then
    source /usr/share/bash-completion/completions/colcon
else
    echo "colcon completion script not found."
fi
