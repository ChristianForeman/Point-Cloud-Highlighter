# Point Cloud Selector
This project allows the user to select a spherical region of a point cloud which
allows the user to run further code on the selected region. It takes in a rosbag as
an argument and gives the user an interface to select the frame of the rosbag they
want to select from. The user can move the interactive marker and change the radius
of the marker to select that spherical region of the point cloud.

## Two Methods to run:
```
roslaunch launch/offline_fitting.launch
python src/pc_selector.py <frame_in_rviz> <default_radius> <bag_filepath>
```
or
```
# Make sure to uncomment the python part of the launch file in offline_fitting.launch
roslaunch launch/offline_fitting.launch frame_id:=<frame_in_rviz> default_radius:=<default_radius> bag_filepath:=<bag_filepath>
```

## Example Output
* [Example 1](images/example1.png)
* [Example 2](images/example2.png)