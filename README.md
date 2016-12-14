# Intrinsic and Extrinsic Calibration
For an overview of obtaining the intrinsic and extrinsic camera calibration
parameters, generating an AruCo pattern, and constructing the calibration
board, visit the
[UNCC Visionlab wiki](http://visionlab.uncc.edu/dokuwiki/ros_and_camera_calibration).

## Installation
Once your AruCo pattern is generated, and the board is constructed, install the [`calibrate_mocap_and_camera`](https://git.antcenter.net/awillis/calibrate_mocap_and_camera) package:

    git clone https://git.antcenter.net/awillis/calibrate_mocap_and_camera.git

### Dependencies
This package depends upon:
  1. `openni2_launch`
  2. `openni2_camera`
  3. `ar_sys`
  4. `image_pipeline`
  5. `ros_vrpn_client`
  
For the XBOX One Kinect (kinectv2), two additional packages are required:
  6. `libfreenect2`
  7. `iai_kinect2`
  
For installation on Ubuntu 14.04 LTS using ROS Indigo, `openni2_launch`, `openni2_camera`, and `ar_sys` were available through the package manager:

    sudo apt-get install ros-indigo-openni2-launch ros-indigo-openni2-camera ros-indigo-ar-sys

[`image_pipeline`](http://wiki.ros.org/image_pipeline) and [`ros_vrpn_client`](https://git.antcenter.net/Pathfinder3/ros_vrpn_client) were cloned into the workspace's source folder.

    git clone https://git.antcenter.net/Pathfinder3/ros_vrpn_client.git
    git clone https://github.com/ros-perception/image_pipeline.git

If you are using the XBOX One Kinect, and you are using opencv 3, the package `iai_kinect2` will not compile. You will need to compile against a different version of opencv. This can be accomplished by editing CMAKELists.txt in `kinect2_bridge`. Change:

    find_package(OPENCV REQUIRED)
to

    find_package(OPENCV 2.4.8)

## Intrinsic Calibration
Intrinsic calibration refers to the estimation of the parameters intrinsic to the camera that affect the imaging process, such as focal length, image center, image sensor format, distortion, and skew.

For intrinsic calibration of the RGB camera, run the launch file
**calibrate_rgbd_rgb_cam.launch** from the `calibrate_mocap_and_camera` package:

    roslaunch calibrate_mocap_and_camera calibrate_rgbd_rgb_cam.launch

For intrinsic calibration of the IR camera, run the launch file
**calibrate_rgbd_ir_cam.launch** from the `calibrate_mocap_and_camera package`:

    roslaunch calibrate_mocap_and_camera calibrate_rgbd_ir_cam.launch

**Note:** Only RGB intrinsic calibration needs to be performed for extrinsic calibration.

### Calibration
1. Rotate and shift the board in the camera's field of view until the progress bars for x, y, skew, and size are green.

2. Once the bars are filled, the option to generate the intrinsic calibration parameters becomes
available.

3. Click **Calibrate**, and wait a moment for the process to complete. The output terminal should show the intrinsic parameters and the **Save** button should become highlighted.

4. Click **Save**, a calibration package (.tar.gz) should be created that contains the images used in calibration as well as the intrinsic parameters in the form of a .yaml file.

5. Click **Commit**, this will save a .yaml file with the intrinsic camera parameters in `~/.ros/camera_info` by default.

## Extrinsic Calibration
Extrinsic calibration refers to the estimation of the coordinate system transformations used to relate the world coordinate frame to the camera coordinate frame. Extrinsic parameters will consist of both a translation and rotation matrix.

### Setup
1. Connect the computer that will be running the calibration launch file to **Pathfinder**, and ensure the Motive software is connected with IP **192.168.1.105**.

2. In Motive, open the project
**Motive Project 2015-12-09 09.37.25 AM.ttp**.

3. The rigid body for the
AruCo calibration board should be recognized as **tf_calib**. Select the marker that will be used to calibrate the camera and create a rigid body from it named **tf_cam**.

  **Note:** It may be beneficial to zero the camera's orientation in Motive while facing the board.

4. Once rigid bodies are defined, modify the argument **calibration_data_filename** in the launch file `board_moves_w_truth.launch` to point to the file  where you would like to store calibration transform data.

5. Run the launch file "board_moves_w_truth.launch" from the package
`calibrate_mocap_and_camera`.

  roslaunch calibrate_mocap_and_camera board_moves_w_truth.launch

### Calibration
1. Orient the the AruCo pattern as shown below and place in in the center of the camera's field of vision.

    <img src="/data/single/pose_calib_00.png?raw=true" width="300">

2. Leave the board stationary, and slowly move the camera.

3. Be sure to exercise all degrees of orientation and position while keeping the majority of the AruCo marker in view.

4. Calibration should take ~1-2 minutes. When ample data has been captured, kill the node.

### Processing Transform Data
The transform data must have outliers/discontinuities removed before being used to calculate extrinsic parameters. This data will be plotted and outliers removed manually.

A Matlab script will be used to plot transform data and determine outliers. If Matlab is not available, Ocatave can be used instead.

1. Ensure Ocatve is installed:

        sudo apt-get install octave

2. Add the Quaternion Getter function by saving the following code in a file with a .m extension:
```
  function R = qGetR( Qrotation )
  % qGetR: get a 3x3 rotation matrix
  % R = qGetR( Qrotation )
  % IN:
  %     Qrotation - quaternion describing rotation
  %
  % OUT:
  %     R - rotation matrix
  %     
  % VERSION: 03.03.2012
  w = Qrotation( 1 );
  x = Qrotation( 2 );
  y = Qrotation( 3 );
  z = Qrotation( 4 );
  Rxx = 1 - 2*(y^2 + z^2);
  Rxy = 2*(x*y - z*w);
  Rxz = 2*(x*z + y*w);
  Ryx = 2*(x*y + z*w);
  Ryy = 1 - 2*(x^2 + z^2);
  Ryz = 2*(y*z - x*w );
  Rzx = 2*(x*z - y*w );
  Rzy = 2*(y*z + x*w );
  Rzz = 1 - 2 *(x^2 + y^2);
  R = [
      Rxx,    Rxy,    Rxz;
      Ryx,    Ryy,    Ryz;
      Rzx,    Rzy,    Rzz];
```

3. Place this file in the same directory as the `calib_analysis.m` file, inside a folder called **"quaternions"**.

4. Edit the `calib_analysis.m` file, after `addpath(quaternions)`, add the name of your transforms file without file extension and save.

5. Edit the transforms file:
    - add the first line:
          tf_cam_to_rgb_optical_calibration_data = [
    - add the last line:
          ]
    - Save the file with a .m extension

6. Run the `calib_analysis.m` script:

        octave --persist calib_analysis.m

    Plots for various pose parameters should appear.

7. Investigate these plots and identify discontinuities as shown below.

    <img src="/doc/Extrinsic TF Outlier Example.png?raw=true">

8. These discontinuities must be removed in order to achieve optimal extrinsic calibration. Determine the index of the discontinuity from the plot and comment out the corresponding line(s) in the transform data file.

    <img src="/doc/Extrinsic TF Data Removal.png?raw=true" width="500">

9. Save the transforms file and rerun `calib_analysis.m`.

    <img src="/doc/Extrinsic TF Outlier Removed.png?raw=true">

    The discontinuity should now be removed. Repeat this process for all discontinuities.

10. After removing all discontinuities and running `calib_analysis.m` once more, the extrinsic parameters can be found in the script output in the form of a translation vector and rotation matrix (quaternion).

    <img src="/doc/Extrinsic Calib Console.png?raw=true">
