[![main](https://github.com/fmrico/perception_asr_stressoverflow/actions/workflows/main.yaml/badge.svg)](https://github.com/fmrico/perception_asr_stressoverflow/actions/workflows/main.yaml)

# perception_asr

These are examples of Image and PointCloud2 perception from HSV filters and Darknet ROS.

* HSV Filter with CV slides [sensor_msgs/msg/Image] -> [vision_msgs/msg/Detection2DArray]
* Darknet ROS detector [darknet_ros_msgs/msg/BoundingBoxes] -> [vision_msgs/msg/Detection2DArray]
* 2D to 3D based on PointCloud2 [vision_msgs/msg/Detection2DArray, sensor_msgs/msg/PointCloud2] -> [vision_msgs/msg/Detection3DArray]
* 2D to 3D based on Depth [vision_msgs/msg/Detection2DArray, sensor_msgs/msg/Image, sensor_msgs/msg/CameraInfo] -> [vision_msgs/msg/Detection3DArray]

![Captura desde 2023-02-28 00-49-42](https://user-images.githubusercontent.com/3810011/221715529-7b96a22b-cf17-4895-98fd-031609979805.png)
