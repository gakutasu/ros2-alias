# ROS2 default workspace path
ROS_WS=~/ros

source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'

# Source the default ROS2 workspace
source $ROS_WS/install/setup.bash
export _colcon_cd_root=$ROS_WS

# Source colcon functions
source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Source the ROS2 workspace in the current directory
function colcon_source() {
    local dir=$(pwd)
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/install/setup.bash" ]; then
            echo "Sourcing ROS2 workspace in $dir"
            source "$dir/install/setup.bash"
            return
        fi
        dir=$(dirname "$dir")
    done
    echo "Current directory is not a ROS2 workspace."
}

# alias
alias kill_ros_processes='ps aux | grep ros | grep -v grep | awk '"'"'{ print "kill -9", $2 }'"'"' | sh'
alias colcon_clean='rm -rf $ROS_WS/install log build'
alias colcon_build='colcon build --symlink-install --parallel-workers $(nproc)'
alias colcon_bt='CURRENT_DIR=$(pwd) && PKG_NAME=`grep "<name>" package.xml | sed -e "s/<[^>]*>//g"` && cd ~/ros && colcon build --parallel-workers $(nproc) --symlink-install --packages-select $PKG_NAME ; cd $CURRENT_DIR'
alias rosdep_install='rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y'
