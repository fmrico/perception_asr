# perception_asr_stressoverflow

[![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-violet)](https://releases.ubuntu.com/22.04/)
[![distro](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)](https://isocpp.org/)
[![main](https://github.com/dgarcu/perception_asr_stressoverflow/actions/workflows/main.yaml/badge.svg)](https://github.com/dgarcu/perception_asr_stressoverflow/actions/workflows/main.yaml)

<p align="center">
  <img src="https://raw.githubusercontent.com/kobuki-base/kobuki_core/devel/resources/kobuki.png"/>
</p>

## Goal 🎯

With this package we aim to the perception process of [seekandcapture-stressoverflow](https://github.com/Docencia-fmrico/seekandcapture-stressoverflow), from detecting a person and publish its location relative to any other frame in a TF.

## Installation 💾

### Main requirements ✅

1. First of all, we need [Ubuntu](https://ubuntu.com/). It is a Linux Debian-based operating system, A.K.A Linux Distro. We are working with the **[22.04 Jammy Jellyfish 🪼](https://releases.ubuntu.com/22.04/)** version. You might be already familiar with it, however [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) you have a step-by-step tutorial on how to install it on your machine.

2. [ROS](https://www.ros.org/). ROS stands for *Robot Operating System*, and it is the **middleware** that the vast majority of robots out there use nowadays, and so are we! We are focusing on the latest stable release, which is **[ROS 2 Humble Hawksbill 🐢](https://docs.ros.org/en/humble/index.html)**. From there you can navigate through all the documentation, but [here](https://docs.ros.org/en/humble/Installation.html) is a shorcut to the installation page.

> **FUN FACT**
>
> Did you know that every ROS distribution is named alphabetically? Not only that, but it is always a turtle species which starts with the current letter. Hawksbill is the colloquial name for the turtle species *Eretmochelys imbricata*. [Here](https://en.wikipedia.org/wiki/Hawksbill_sea_turtle) you can learn more about this animal if you feel curious!

3. **[ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) package**. [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) is research group from the [Universidad Rey Juan Carlos](https://www.urjc.es/). They have developed the package with all the dependencies you will need, among some other things, like a simulator setup with [Gazebo](https://gazebosim.org/home). You can find the installation steps on their README, check it out!

### How to use 💭

This package **is meant to be launched from [seekandcapture-stressoverflow](https://github.com/Docencia-fmrico/seekandcapture-stressoverflow)**. However there are a few details that we have to tweak before *being launched anywhere*.

If you came here from [seekandcapture-stressoverflow](https://github.com/Docencia-fmrico/seekandcapture-stressoverflow), this package and its dependecies should be already cloned and built. The source code should be in:

```bash
<your-workspace-path>/src/ThirdParty/perception_asr_stressoverflow
```

As well as:

```bash
<your-workspace-path>/src/ThirdParty/darknet_ros_yolov4
<your-workspace-path>/src/ThirdParty/ros2_v4l2_camera
```

[`depth.launch.py`](./launch/depth.launch.py) has a parameter in [`params.yaml`](./config/params.yaml) where you can specify if this launcher is also launching the `darknet_ros` package.

```yaml
# Options:
#
# launch_darknet: true - false

perception_asr_stressoverflow:
  launchers:
    launch_darknet: true
```

You might want to deactivate this feature since `darknet_ros` will keep your terminal window busy with its output, making debugging impossible. However, if you leave this set to `true` you should make this change in [`darknet_ros.launch.py`](https://github.com/Ar-Ray-code/darknet_ros/blob/df79053480df97db0a972d8b2c9beeadb1126e7f/darknet_ros/launch/darknet_ros.launch.py)

```python
darknet_ros_cmd = Node(
    package='darknet_ros',
    executable='darknet_ros',
    name='darknet_ros',
    output='screen',
    parameters=[ros_param_file, network_param_file,
      {
        "config_path": yolo_config_path, 
        "weights_path": yolo_weights_path,
      }],
    remappings=[('image_raw', image) # This is the line you should add!
    ])
```

That line **allows us to remap the input image topic from a [launcher parameter](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#setting-parameters-in-the-launch-file)**. Otherwise, the *mother of all launchers* will not work propperly!

Remember as well, if you launch `darknet_ros.launch.py` manually, you should add also manually the remapped topic:

```bash
ros2 launch darknet_ros darknet_ros.launch.py 'image:=<your-rgb-img-topic>'
```

> **IMPORTANT NOTE**. The topic should not start with a forward slash (`/`).
> `'image:=/example/topic'` -> `'image:=example/topic'`

**Our launcher will also handle the simulation remap**. Make sure that you have correctly set [ir_robots/config/params.yaml](https://github.com/IntelligentRoboticsLabs/ir_robots/blob/60e610266f7be3d630684710eb8f18786034dda1/config/params.yaml) for the test you are going to run!

## Implementation ⚙️

It takes three main steps:

1. All procedures can be summarized in transformation from one topic message to another. We are using [Darknet](https://github.com/Ar-Ray-code/darknet_ros_fp16), whose node publish this message:

   `darknet_ros_msgs/msg/BoundingBoxes`

   They consist on the *Bounding box* characteristics (Position, size...) of a detected object, with the `class_id` matching the detected object, like `person`, `chair`... `banana` [*not for scale*](https://knowyourmeme.com/memes/banana-for-scale).

   From there, we need to *pack* that information provided by `darknet_ros` in a more convenient format:
  
   `vision_msgs/msg/Detection2DArray`
  
   This format is easier to work with for the next node we need. In this format, already appears information about the position and orientation from the object detected, but we need to add more data to actually fullfil the real position in space of said object. This whole procedure takes place in [`DarknetDetectionNode.cpp`](./src/perception_asr_stressoverflow/DarknetDetectionNode.cpp).
  
2. This step has two alternatives:
   1. **Point cloud**. Which will not be covered by now. Sorry for the inconvenience!
   2. **Depth**. Along with data from the camera sensor itself (Those kind of specs that only camera-freaks could tune) **we can actually calculate the depth of any given pixel of the image**... Take a guess on which pixel we are choosing for calculate its depth.
   
   Exactly, **the pixel that represents the center of the bounding box** retrieved by Darknet! This is how we can triangulate the position of the robot relative to a detected object with a [humble piece of hardware](https://shop.orbbec3d.com/Astra) and... Tons of software, actually. Who are we kidding.

   Taking the data from this three messagges together:
   
   `vision_msgs/msg/Detection2DArray`
   `sensor_msgs/msg/Image`
   `sensor_msgs/msg/CameraInfo`
   
   We can make a message from this type:
   
   `vision_msgs/msg/Detection3DArray`

    Which is an array with all detected objects in the image along with its position relative to the camera. The last step is to use this information to calculate the position relative to the robot, since **we actually know the position of the camera relative to the robot**. *Awesome!*
    
    This whole procedure takes place in [`DetectionTo3DfromDepthNode.cpp`](./src/perception_asr_stressoverflow/DetectionTo3DfromDepthNode.cpp).
    
3. Once we got the information described earlier, we can take advantage of the [**TF system from ROS 2**](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) to calculate the real position in space from the detected object relative to the robot. *Just from an image!*
   
   There are actually **two** nodes using this:
      1. [`DetectedPersonMonitor.cpp`](./src/perception_asr_stressoverflow/DetectedPersonMonitor.cpp), which publishes a marker from the robot to the detected person, so we can debug the output from the whole procedure easily in [RViz](https://turtlebot.github.io/turtlebot4-user-manual/software/rviz.html).
      2. [`DetectedPersonTfPub.cpp`](./src/perception_asr_stressoverflow/DetectedPersonTfPub.cpp), which publishes the TF (**T**rans**F**orm 😉) of the detected person. Here is also where we filter the detected objects to only publish TF's for `person`s.
      
## Observations 🔎

- There was a **bug** in [ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) package, where the frame `base_footprint` is not set correctly, thus the tf calculation from the detected person to the robot could not be possible. However, using `base_link` does the job despite of being lifted a few milimeters from ground, since we are only using a 2D plane to maneuver the robot. However, the problem becomes even worse in the simulation eviroment. More details on [simulation test](#simulation-%EF%B8%8F) section.

- *Continous Integration* (CI) setup. We have added a workflow to be triggered through **GitHub Actions** whenever a `pull request` is made. From this workflow, the code is built and tested in any enviroement we want[^1]. You can find this workflow [here](./.github/workflows/colcon.yaml). With this feature we can automatically test our code before pushing it to the `main` branch. This allows us to directly review the `pull request` without worrying about breaking the already pushed code, coding style... Since we will instantly see a checkmark with the test output. A further step that can be taken is to make this test to trigger another workflow that makes our work easier, like automatically deploy our packet! A.K.A. *Continous Deployment* (CD).

  In order to tackle this, we have followed [this article](https://ubuntu.com/blog/ros-2-ci-with-github-actions) from the Ubuntu blog itself. There you can find more details about how exactly the [workflow file](./.github/workflows/colcon.yaml) actually works!

### Tests 🧾

We have checked the correct behaviour from the nodes by looking at the related topics in [RViz](https://github.com/ros2/rviz)

#### Simulation 🖥️

Before launching the program in the real robot, **we have tested it beforehand in a simulated enviroment**, using tools like [Gazebo](https://gazebosim.org/home) or [RViz](https://github.com/ros2/rviz). We were skeptical about `darknet_ros` capabilities to recognise a virtual person. Luckily for us, we were pleased by a fantastic TF and a Marker successfully published!

> NOTE: We used `simple_office_with_people` map from [ir_robots/config/params.yaml](https://github.com/IntelligentRoboticsLabs/ir_robots/blob/60e610266f7be3d630684710eb8f18786034dda1/config/params.yaml).

https://user-images.githubusercontent.com/92941081/227732563-009c99f2-180a-49ea-bc28-79fe884209ff.mp4


> **IMPORTANT NOTE**: Some TF's on simulation enviroment are not properly set, thus the apparent erratic behaviour in the simulation video.

Time for the real robot 😁

#### Real World 🌍

## About

This is a project made by the [StressOverflow], a student group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2023.

Maintainers:

- [Carlos Escarcena]
- [Arantxa García]
- [Diego García]
- [Teresa Ortega]

## License

[![License](https://img.shields.io/badge/License-Apache_2.0-yellowgreen.svg)](https://www.apache.org/licenses/LICENSE-2.0) 

This work is licensed under a [APACHE LICENSE, VERSION 2.0][apache2.0].

[apache2.0]: https://www.apache.org/licenses/LICENSE-2.0

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[StressOverflow]: https://github.com/orgs/Docencia-fmrico/teams/stressoverflow
[Carlos Escarcena]: https://github.com/cescarcena2021
[Arantxa García]: https://github.com/arantxagb
[Diego García]: https://github.com/dgarcu
[Teresa Ortega]: https://github.com/mtortega2021

[^1]: In this case, Ubuntu 22.04 Jammy Jellyfish and ROS 2 Humble Hawskbill, which is the same enviroment we are working with.
