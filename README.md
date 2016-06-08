# Intrinsic and Extrinsic Calibration
For an overview of obtaining the intrinsic and extrinsic camera calibration
parameters, generating an AruCo pattern, and constructing the calibration
board, visit the
[UNCC Visionlab wiki](http://visionlab.uncc.edu/dokuwiki/ros_and_camera_calibration).

## Installation
Once your AruCo pattern is generated, and the board is constructed, install the [`calibrate_mocap_and_camera`](https://git.antcenter.net/awillis/calibrate_mocap_and_camera) package:

    git clone <fill in here>

### Dependencies
This package depends upon:
  1. `openni2_launch`
  2. `openni2_camera`
  3. `ar_sys`
  4. `image_pipeline`
  5. `ros_vrpn_client`

For installation on Ubuntu 14.04 LTS using ROS Indigo, `openni2_launch`, `openni2_camera`, and `ar_sys` were available through the package manager:

    sudo apt-get install ros-indigo-openni2-launch ros-indigo-openni2-camera ros-indigo-ar-sys

[`image_pipeline`](http://wiki.ros.org/image_pipeline) and [`ros_vrpn_client`](https://git.antcenter.net/Pathfinder3/ros_vrpn_client) were cloned into the workspace's source folder.

    git clone https://git.antcenter.net/Pathfinder3/ros_vrpn_client.git
    git clone https://github.com/ros-perception/image_pipeline.git

## Intrinsic Calibration
For intrinsic calibration of the RGB camera, run the launch file
**calibrate_rgbd_rgb_cam.launch** from the `calibrate_mocap_and_camera` package:

    roslaunch calibrate_mocap_and_camera calibrate_rgbd_rgb_cam.launch

For intrinsic calibration of the IR camera, run the launch file
**calibrate_rgbd_ir_cam.launch** from the `calibrate_mocap_and_camera package`:

    roslaunch calibrate_mocap_and_camera calibrate_rgbd_ir_cam.launch

**Note:** Only RGB intrinsic calibration needs to be performed for extrinsic calibration.

### Calibration

1. Rotate and shift the board in the camera's field of view until the progress bars for x, y, skew and size are green.

2. Once the bars are
filled, the option to generate the intrinsic calibration parameters becomes
available.

3. Click calibrate, save, and commit to obtain the parameters.

By default, the intrinsic parameters are stored in a .yml file that can be
found in ~/.ros/camera_info/

## Extrinsic Calibration
Connect the computer that will be running the calibration launch file to
nrcnet.
<br><br>
In the mocap software, open the project
"Motive Project 2015-12-09 09.37.25 AM.ttp". Name the rigid body for the
AruCo calibration board as "tf_calib" and the rigid body for the camera as
"tf_cam".
<br><br>
Once the mocap rigid bodies are defined, modify the argument
"calibration_data_filename" in the launch file "board_moves_w_truth.launch"
to the directory/filename.txt where the calibration transforms will be
stored.
<br><br>
Run the launch file "board_moves_w_truth.launch" from the package
calibrate_mocap_and_camera.
<pre>
<style type="text/css">
code { background-color: #CBCBCB; }
</style>
<code>$ roslaunch calibrate_mocap_and_camera board_moves_w_truth.launch
</code>
</pre>
To obtain the calibration translations. Orientate the top left corner of the
pattern so that it is located in the top left of the camera's field of vision.
Leave the board stationary, and move the camera.
</p>

<h3> Processing Transform Data </h3>

<p>
Ensure Ocatve is installed:
<pre>
<style type="text/css">
code { background-color: #CBCBCB; }
</style>
<code>$ sudo apt-get install octave
</code>
</pre>
Add the Quaternion Getter function by saving the following code in a file with a .m extenstion:
<a href="http://www.mathworks.com/matlabcentral/fileexchange/35475-quaternions/content/qGetR.m ">
  here.</a>
</p>
Place this file in the same directory as the calib_analysis.m file, inside a
folder called "quaternions".

Edit the calib_analysis file:

After "addpath(quaternions)", add the name of your transforms file without file extension



In the transforms file add the first line:

tf_cam_to_rgb_optical_calibration_data = [

And the last line:

]

Save the file with a .m extension