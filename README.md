<div align="center">
    <h1>Simple Ros2 segmentation node based on z heigh</h1>
</div>

## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/leonardonels/vertical-segmentation.git
```
```commandline
cd ~/ros2_ws
colcon build --symlink-install --packages-select vertical-segmentation
source install/setup.bash
```

## :abacus: Parameters
```yaml
pointcloud_filter_node:
  ros__parameters:

    input_topic: "/lidar/filtered"
    output_topic: "/lidar/segmented"

    intensity: true  # If set to false, the intensity will be discarded from the pcl
    
    # The number of vertical zones can change, for this example there is only one
    vertical_zones:
      - "start: 0.09, end: 1.0, downsample: 1"

```
