# tk23_vision

## Kinect setup (22.04)

Follow [Azure Kinect with ROS2 Humble - Robotics Stack Exchange](https://robotics.stackexchange.com/questions/24529/azure-kinect-with-ros2-humble).

1. Install the Azure Kinect Sensor SDK ([reference](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1790)):
   
   * Temporarily switch the source list of apt to 20.04:
     
     * replace the file `/etc/sources.list` with [20.04 source list](https://gist.github.com/ishad0w/788555191c7037e249a439542c53e170)
     
     * run `sudo apt update` and then install `libsoundio1`
     
     * switch back your source list to 22.04
   
   * For Ubuntu 22.04, you can download [k4a-tools_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb), [libk4a1.4-dev_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb) and [libk4a1.4_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb) for 18.04.

2. (done) Clone [Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/humble) to the `src` folder and checkout to the humble branch.

3. run rosdep `rosdep install --from-paths src --ignore-src -r -y`

4. build `colcon build` and `source install/setup.bash`

5. Copy the file [99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules) into `/etc/udev/rules.d/`, run `sudo udevadm control --reload` to reload settings, then detach and reattach the kinect

6. Run the launch file `ros2 launch azure_kinect_ros_driver driver.launch.py`

## realsense-ros 

Follow ([reference](https://github.com/tinkerfuroc/tk23_vision/blob/main/src/realsense-ros/README.md))

## tinker_vision_msgs

Try to build the package:

```shell
colcon build --packages-select tinker_vision_msgs
```

If the command fails, install the dependencies and try again:

```shell
pip install empy lark catkin_pkg numpy
```

## object_detection

Install the package to the system python. I've tried to use conda packages in ROS2 nodes but failed.

```shell
pip install ultralytics
```
## face_recongition_arcface

 Using Arcsoft SDK to complete face recongition task.

 1. First use Kinect or Realsense Camera for the input, and chose the correct subscribed image topic name according to the camera you use. ```class ReceptionistFace()``` is defined in ```
 /face_recognition_arcface/face_recognition_arcface/receptionist_face_task.py```, and the topic name configuration is in it.

 2. (*) [Arcsoft face SDK](https://ai.arcsoft.com.cn/product/arcface.html) : Register an account and get APPID and SDKKEY for face recognition SDK, and change APPID and SDKKEY in /face_recognition_arcface/receptionist_face_task.py.

 3. (*) Test the SDK demo: Copy folder /lib altogether (with two .so file right in it) in SDK you download, put under /face_recognition_arcface/arcface_demo, then run this to test: 

```python 
python demo.py
```

 4. Run the face recognition node:
    
```shell
ros2 run face_recognition_arcface receptionist_face_task.py
```

 5. Call the face registeration service:

```shell
ros2 service call /vision/face/register tinker_vision_msgs/FaceRegister "{state: 0}"
```
Call the face recognition service:
```shell
ros2 service call /vision/face/register tinker_vision_msgs/FaceRegister "{state: 1}"
```


## Tasks

### Carry My Luggage

1. Identify the bag pointed at by the operator

2. Identify and follow the operator to his car

3. (Optional) return to the house

### Receptionist

1. Register the host's and the guests' faces and info 

2. Identify the two guests (name, favorite drink, color of clothes, color of hair, gender, age, etc.)

3. Find empty chairs for guests
