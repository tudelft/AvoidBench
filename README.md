# Avoidbench
AvoidBench: A high-fidelity vision-based obstacle avoidance benchmarking suite for multi-rotor.

# 1. Introduction
This work is based on [Flightmare](https://github.com/uzh-rpg/flightmare). In this project, we propose AvoidBench as a benchmarking suite for evaluating the performance of vision-based obstacle avoidance algorithms. We choose Flightmare as the basic backbone of AvoidBench, because it is lighter and can achieve higher simulation speed than Airsim. Based on Flightmare, we have for now built two simulation scenes for benchmarking: a forest environment and an indoor environment. It is easy to change the distribution of obstacles and complexity of map so that researchers can reveal the potential of drones using their algorithms. And we propose a complete set of metrics which contain the flight performance and environment complexity to evaluate the obstacle avoidance algorithms.

<p align="center">
  <img src="https://github.com/tudelft/AvoidBench/blob/main/src/images/Picture1.gif"/>
</p>

<p align="center">
  <img src="https://github.com/tudelft/AvoidBench/blob/main/src/images/framework.PNG"/>
</p>

# 2. Installation
## 2.1 Install with ROS
This project is developed by on Ubuntu20.04 with ROS noetic. We kindly recommend you to use the same ROS version if you want to test your algorithms in AvoidBench. For gcc and g++ version, we test both 7.5.0 and 9.4.0. You can check this by typing in a terminal ``` gcc --version ``` and ``` g++ --version ```. Follow this [guide](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) if your compiler is not compatible.

For the hardware, we suggest you testing AvoidBench in a PC or laptop with GPU. The rendering system Unity3D can work much better than without a NVIDIA discrete GPU. The depth map from stereo vision also needs accelerated computing by CUDA.

Run the following commands to setup:
``` bash
# install Open3D
sudo apt update
sudo apt install git libtool build-essential cmake
git clone --recursive -b v0.9.0 https://github.com/isl-org/Open3D.git
cd Open3D
mkdir build
cd build
cmake ..
make -j
sudo make install
```
``` bash
git clone https://github.com/tudelft/AvoidBench.git
sudo apt update
sudo apt install libzmqpp-dev libopencv-dev unzip python3-catkin-tools
sudo apt install libgoogle-glog-dev protobuf-compiler ros-noetic-octomap-msgs ros-noetic-octomap-ros python3-vcstool
cd Avoidbench
echo "export AVOIDBENCH_PATH=path_to_this_project/AvoidBench/src/avoidbench" >> ~/.bashrc
catkin build
```

## 2.2 Install with docker
Now we have docker image for AvoidBench!
Follow this [guide](https://docs.docker.com/engine/install/ubuntu/) to install docker. The following steps are used to make your NVIDIA graphics device can work with docker container:

```bash
# in terminal of the host computer
xhost +
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Then get our docker image and run a container:

```bash

# in terminal of the host computer
sudo docker pull hangyutud/noetic_avoidbench:latest
sudo docker run -it --device=/dev/dri --group-add video --volume=/tmp/.X11-unix:/tmp/.X11-unix  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --gpus all --name=noetic_ab -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics -e NVIDIA_VISIBLE_DEVICES=all hangyutud/noetic_avoidbench:latest /bin/bash
# in terminal of the docker container
cd AvoidBench
catkin build
```


# 3. Task
Different from other benchmarks for drones, AvoidBench focus on the **evaluation of obstacle avoidance performance in environments with different comlexity**. Taking the outdoor scene parameter file [task_outdoor.yaml](https://github.com/tudelft/AvoidBench/tree/main/src/avoidbench/avoid_manage/params/task_outdoor.yaml) as an example, the following parameters define the evaluation task:

``` bash
flight_number: 30 #the number of maps, randomly generate 30 maps with different comlexity
mission:
  trials: 10 # 10 times flight for each map, each trial has randomly generated start and goal points
  seed: 32 # random seed for maps and start and end points
  start_area: [40, 6.0] # we generate start and end points at the four edges of the the squre map, *start area* and *start origin* define the start areas 
  start_origin: [0, 3.0]
  radius_area: 3.5 # the range of the radius for Poisson Distribution of obstacles
  radius_origin: 2.5 # the minimum value of the radius for Poisson Distribution of obstacles
  end_area: [40, 6.0] # *end area* and *end origin* define the end areas
  end_origin: [0, 34]
  pc_file_name: pointcloud-unity # name of saved point cloud file
```
Here we show the environments with different comlexity:

<p align="center">
  <img src="https://github.com/tudelft/AvoidBench/blob/main/src/images/Media1.gif" width = "640" height = "360"/>
</p>

# 4. Usage
To use this benchmark, you should download the Unity file first. You can download the Unity [standalone](https://github.com/tudelft/AvoidBench/releases/download/v0.0.2/AvoidBench.zip). **OR** you can just run the following commands to setup:
(For docker version, we have already done this, so this step is only for ROS version.)

``` bash
# now you are in your ros workspace
cd src/avoidbench/unity_scene/
wget "https://github.com/tudelft/AvoidBench/releases/download/v0.0.2/AvoidBench.zip"
unzip -o AvoidBench.zip
rm AvoidBench.zip
```
Then go back to the ros workspace and run ros launch file
``` bash
cd ../../..
source devel/setup.bash
# run the launch file
roslaunch avoid_manage rotors_gazebo.launch
```
We also wrapped the communication interface between Unity and C++ to a python version by [pybind11](https://pybind11.readthedocs.io/en/stable/) and rewrote the avoid_manage node by python. If you want to test your python code, maybe this version is better for you to understand the whole task process. Of course, this version still need the dependency on ROS/rospy.
``` bash
# optional
roslaunch avoid_manage test_py.launch
```
Now you have already run the Unity3D and ros nodes for RotorS, [RPG controller](https://github.com/uzh-rpg/rpg_quadrotor_control), and *avoid_manage* (if there's no error happen).

Always keep in mind that AvoidBench can provide stereo images, real depth images (generated from Unity, but we don't suggest you use these since there is no noise which is always far from real condition), SGM depth images (calculated by stereo vision). You can also get the state of the drone (groud truth). Then you need to know some special ros topics and use them to implement your own obstacle avoidance algorithms. **PLEASE read it carefully if you want to use AvoidBench!**

```bash 
# These topics are provided for users
/depth # depth image calculated by SGM, type: mono16
/rgb/left # left image, type: bgr8
/rgb/right # right image, type: bgr8
/hummingbird/camera_depth/camera/camera_info # camera parameters of depth image
/hummingbird/ground_truth/odometry # odometry topic of the drone, velocity is in bodyframe
/hummingbird/ground_truth/imu # imu topic of the drone
/hummingbird/goal_point # the goal point for each trial
/hummingbird/task_state # task state topic, Mission_state: 0(initial state of the whole program); 1(setting the Unity scenes); 2(sending the goal point topic); 3(waiting for the control command); 4(receving the control command and flying); 5(reseting the gazebo for other trial), DONOT send any control command on periods 0, 1, 5
/hummingbird/autopilot/pose_command # position control command. X, Y, Z and yaw
/hummingbird/autopilot/velocity_command # velocity control command. Vx, Vy, Yz and yaw
/hummingbird/autopilot/reference_state # reference state control command. pose, velocity, accelerarion, heading, heading rate, jerk
/hummingbird/autopilot/control_command_input # bodyrates and collective mass normalized thrust [m/s^2]
```
 If you want to know more about the control mode, refer to [RPG controller](https://github.com/uzh-rpg/rpg_quadrotor_control). And here are one topic that users need to provide to AvoidBench (evaluate the efficiency of your obstacle avoidance algorithms).
 ```bash
  # This topic needs to be provided by users
  /hummingbird/iter_time # Computer running time required for each trajectory generation
 ```
 
# 5. Demo of Benchmarking Agile_autonomy

We provide a demo of benchmarking [Agile_autonomy](https://rpg.ifi.uzh.ch/AgileAutonomy.html) algorithm by AvoidBench. The benckmark version can be found [here](https://github.com/NPU-yuhang/agile_autonomy). Following these steps to build and run this demo:

```bash 
# in the ROS workspace of AvoidBench
cd src/
git clone --recursive https://github.com/NPU-yuhang/agile_autonomy.git
cd ..
catkin build
```
Create your learning environment (install [conda](https://docs.conda.io/en/latest/miniconda.html) if you want to use Agile-autonomy)

```bash 
source devel/setup.bash
roscd planner_learning
conda create --name tf_24 python=3.7
conda activate tf_24
conda install tensorflow-gpu
pip install rospkg==1.2.3,pyquaternion,open3d,opencv-python
```

Fly with Aigle-autonomy, Open a terminal and type:

```bash
source devel/setup.bash
roslaunch agile_autonomy simulation.launch
```

Run the Network in an other terminal:

```bash
source devel/setup.bash
conda activate tf_24
roscd planner_learning
python test_trajectories.py --settings_file=config/test_settings.yaml

```

# 6. Results
We tested several mainstream, state-of-the-art obstacle avoidance algorithms, including: [Agile-Autonomy](https://github.com/uzh-rpg/agile_autonomy)(learning-based), [Ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)(optimization-based), and [MBPlanner](https://github.com/ntnu-arl/mbplanner_ros)(motion-primitive-based, the original version is designed for explorationm, we changed the exploration gain to the distance of current point to the goal point so that the drone can always fly to a fixed goal). Here is the testing video:
<p align="center">
  <img src="https://github.com/tudelft/AvoidBench/blob/main/src/images/Media1_4.gif" width = "640" height = "360"/>
</p>

# 7. Citation
Paper: [AvoidBench](https://arxiv.org/abs/2301.07430)

```
@article{yu2023avoidbench,
  title={AvoidBench: A high-fidelity vision-based obstacle avoidance benchmarking suite for multi-rotors},
  author={Yu, Hang and de Croon, Guido CH and De Wagter, Christophe},
  journal={arXiv preprint arXiv:2301.07430},
  year={2023}
}
```

# 8. Ackowledgements
This work is based on [Flightmare](https://github.com/uzh-rpg/flightmare). Thanks for the great works from [Yunlong Song](https://yun-long.github.io) and [Antonio Loquercio](https://antonilo.github.io/).

Thanks for the helping of all the members in TU-Delft [MAVLab](https://mavlab.tudelft.nl/).

# 9. Licence
The source code is released under MIT license.
