# ROS2 Alias

`ros2-alias` provides useful command aliases for simplifying and speeding up the usage of ROS 2 workflows. With these aliases, you can more easily manage your ROS 2 environment, build processes, and commonly used commands.

## Installation

Follow these steps to set up `ros2-alias`:

```sh
git clone https://github.com/gakutasu/ros2-alias.git
ln -sf $(pwd)/ros2-alias/ros2_alias.sh ~/ros2_alias.sh
echo "source ~/ros2_alias.sh" >> ~/.bashrc
source ~/.bashrc
```

## ROS2 Default Workspace Path

The default workspace path is set to:

```sh
ROS_WS=~/ros
```

You can freely change this path to match your preferred workspace location.

---

### Simplify Your ROS 2 Workflow 🚀
