# Replayer for gonbuki teleop bagfiles

This is meant to test localization solutions for the gonbuki robot, such as `beluga_amcl`.

## Usage

An example to get you going:

```
ros2 launch donatello_rosbag_utils replay_teleop_bags_with_localization_launch.py rosbag_name:=mapping_capture
```

Rosbags need to be located within the `bags` folder in this package. Notice that they will be ignored by git.
