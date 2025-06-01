#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Missing command line argument"
    echo "Usage: $(basename $0) <command>"
    exit 1
fi

mkdir -p $USER_WS/persistence/$(id -u)/ccache
mkdir -p $USER_WS/persistence/$(id -u)/install
mkdir -p $USER_WS/persistence/$(id -u)/build

if [ ! -d "$HOME/.ccache" ]; then
    ln -s $USER_WS/persistence/$(id -u)/ccache  $HOME/.ccache
    ln -s $USER_WS/persistence/$(id -u)/install $USER_WS/install
    ln -s $USER_WS/persistence/$(id -u)/build   $USER_WS/build
fi

# if bash_history is not a symlink or if it does not exist, remove it and create a new symlink
if [ ! -h "$HOME/.bash_history" ]; then
    if [ -f "$HOME/.bash_history" ]; then
        rm $HOME/.bash_history
    fi
    touch $USER_WS/persistence/$(id -u)/.bash_history
    ln -s $USER_WS/persistence/$(id -u)/.bash_history $HOME/.bash_history
fi

. /opt/ros/humble/setup.bash

# Fix to gazebo crashing when loading simulation. It's unclear why this fixes it,
# and why it happens (the same repo version a couple of months ago didn't have this issue,
# so the change is in gazebo or ros). See
# https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/
if [ -f "/usr/share/gazebo/setup.sh" ]; then
    . /usr/share/gazebo/setup.sh
fi

# In my benefit, while I need to build gazebo from source, because the colcon mixin does not
# seem to pick cc1plus
mkdir -p ~/.bin

if [ ! -f ~/.bin/cc1plus ]; then
    ln -s $(which ccache) ~/.bin/cc1plus
    export PATH=~/.bin:$PATH
fi

ros2 daemon start

echo
echo "ROS_DISTRO    :" $ROS_DISTRO
echo "ROS_DOMAIN_ID :" $ROS_DOMAIN_ID
echo

ros2 daemon start

exec "$@"
