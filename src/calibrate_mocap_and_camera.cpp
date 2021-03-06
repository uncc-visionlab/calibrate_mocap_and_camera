/*
 * File:   Feature3DEngine.cpp
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// TF includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Includes for this Library
#include <calibrate_mocap_and_camera/calibrate_mocap_and_camera.h>

#include <eigen_conversions/eigen_msg.h>

namespace Eigen {

    void toString(std::string name, Eigen::MatrixXf mat) {
        static std::string sep = "\n----------------------------------------\n";
        static int StreamPrecision = 4;
        static Eigen::IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        std::cout << sep << name << " = " << mat.format(OctaveFmt) << ";" << sep;
    }
}

void CalibrateMocapAndCamera::tf_camera_marker_Callback(const geometry_msgs::TransformStampedConstPtr& tf_camera_marker_pose) {
    static geometry_msgs::TransformStampedConstPtr first_tf;
    static int num_msgs;
    static tf::Transform initialTransform;
    static Eigen::Vector3f eig_trns;
    static Eigen::Vector4f eig_quat;
    if (!first_tf) {
        first_tf = tf_camera_marker_pose;
        num_msgs = 0;
        initialTransform.setIdentity();
        eig_quat[0] = eig_quat[1] = eig_quat[2] = eig_quat[3] = 0;
        eig_trns[0] = eig_trns[1] = eig_trns[2] = 0;
    }
    //    geometry_msgs::Vector3 translation;
    //    geometry_msgs::Quaternion quaternion;
    //    translation = tf_camera_marker_pose->transform.translation;
    //    quaternion = tf_camera_marker_pose->transform.rotation;
    //    eig_trns[0] += translation.x;
    //    eig_trns[1] += translation.y;
    //    eig_trns[2] += translation.z;
    //    eig_quat[0] += quaternion.x;
    //    eig_quat[1] += quaternion.y;
    //    eig_quat[2] += quaternion.z;
    //    eig_quat[3] += quaternion.w;
    //    num_msgs++;
    //ROS_INFO("Heard tf_camera_marker.");
    //    std::cout << "rotation = " << tf_truth->transform.rotation << std::endl
    //            << "translation = " << tf_truth->transform.translation << std::endl;
    //    if (tf_camera_marker_pose->header.stamp.sec - first_tf->header.stamp.sec > tf_truth_init_time) {
    //        std::cout << "Initialization time has expired." << std::endl;
    //        eig_trns[0] /= num_msgs;
    //        eig_trns[1] /= num_msgs;
    //        eig_trns[2] /= num_msgs;
    //        eig_quat[0] /= num_msgs;
    //        eig_quat[1] /= num_msgs;
    //        eig_quat[2] /= num_msgs;
    //        eig_quat[3] /= num_msgs;
    //        tf::Vector3 tval(eig_trns[0], eig_trns[1], eig_trns[2]);
    //        tf::Quaternion qval(eig_quat[0], eig_quat[1], eig_quat[2], eig_quat[3]);
    //        qval = qval.normalize();
    //        tf::Transform initialTransform(qval, tval);
    //        std::cout << "Received " << num_msgs << " values for ground truth."
    //                << " Setting ground truth transform to: " << std::endl
    //                << "rotation = (" << qval.getX() << ", " << qval.getY()
    //                << ", " << qval.getZ() << ", " << qval.getW() << ")" << std::endl
    //                << "translation = (" << tval.getX() << ", " << tval.getY()
    //                << ", " << tval.getZ() << ")" << std::endl;
    //        setInitialTransform(initialTransform);
    //        sub_tf_cam_marker.shutdown();
    //    }
}

void CalibrateMocapAndCamera::tf_calib_marker_Callback(const geometry_msgs::TransformStampedConstPtr& tf_calib_marker_pose) {
    //ROS_INFO("Heard tf_calib_marker.");

}

void CalibrateMocapAndCamera::ar_calib_pose_Callback(const geometry_msgs::TransformStampedConstPtr& ar_calib_pose) {
    //ROS_INFO("Heard ar_calib_pose.");
    static tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    static geometry_msgs::TransformStampedConstPtr prior_tf;
    static tf::Vector3 prior_axis;

    std::string cam_frame_id_str("tf_cam");
    std::string calib_frame_id_str("tf_calib");
    tf::StampedTransform cam_marker_pose;
    tf::StampedTransform calib_marker_pose;
    tf::StampedTransform tf_cam_to_rgb_optical_frame;
    static bool DEBUG = false;
    if (DEBUG) {
        ROS_INFO("Looking up transform from frame '%s' to frame '%s'", map_frame_id_str.c_str(),
                cam_frame_id_str.c_str());
    }
    //ros::Time queryTime(ros::Time(0));
    ros::Time queryTime = ros::Time::now();
    //ros::Time queryTime(ros::Time::now()-ros::Duration(0.1));
//    try {
//        listener.waitForTransform(map_frame_id_str, cam_frame_id_str,
//                queryTime, ros::Duration(1));
//        listener.lookupTransform(map_frame_id_str, cam_frame_id_str,
//                queryTime, cam_marker_pose);
//        listener.waitForTransform(map_frame_id_str, calib_frame_id_str,
//                queryTime, ros::Duration(1));
//        listener.lookupTransform(map_frame_id_str, calib_frame_id_str,
//                queryTime, calib_marker_pose);
//    } catch (tf::TransformException ex) {
//        //ROS_ERROR("%s", ex.what());
//        //        ros::Duration(1.0).sleep();
//    }
//    if (DEBUG) {
//        std::cout << "cam_pose = " << cam_marker_pose.getOrigin().x() << ", " <<
//                cam_marker_pose.getOrigin().y() << ", " << cam_marker_pose.getOrigin().z() << std::endl;
//        std::cout << "calib_pose = " << calib_marker_pose.getOrigin().x() << ", " <<
//                calib_marker_pose.getOrigin().y() << ", " << calib_marker_pose.getOrigin().z() << std::endl;
//        std::cout << "ar_pose = " << ar_calib_pose->transform.translation << std::endl;
//    }
    static geometry_msgs::TransformStamped ar_calib_to_camera;

    tf::Transform transform;
    //    tf::Vector3 translation(ar_calib_pose->transform.translation.x,
    //            ar_calib_pose->transform.translation.y, ar_calib_pose->transform.translation.z);
    //    translation.setX(-translation.getX());
    //    transform.setOrigin(translation);
    //    tf::Matrix3x3 rotationMatrix(tf::Quaternion(ar_calib_pose->transform.rotation.x, ar_calib_pose->transform.rotation.y,
    //            ar_calib_pose->transform.rotation.z, ar_calib_pose->transform.rotation.w).normalize());
    //    tf::Vector3 e_x = rotationMatrix[0];
    //    e_x *= -1;
    //    rotationMatrix[0] = e_x;
    //    transform.setBasis(rotationMatrix);
//    transform.setOrigin(tf::Vector3(ar_calib_pose->transform.translation.x,
//            ar_calib_pose->transform.translation.y, ar_calib_pose->transform.translation.z));
//    transform.setRotation(tf::Quaternion(ar_calib_pose->transform.rotation.x, ar_calib_pose->transform.rotation.y,
//            ar_calib_pose->transform.rotation.z, ar_calib_pose->transform.rotation.w).normalize());
    //tf::Transform itransform = transform.inverse();
    //br.sendTransform(tf::StampedTransform(itransform,
    //        ros::Time::now(), "ar_optical_frame", "rgb_optical_pose"));
    try {
        //listener.waitForTransform(cam_frame_id_str, "rgb_optical_pose",
        //        queryTime, ros::Duration(1));
        //listener.lookupTransform(cam_frame_id_str, "rgb_optical_pose",
        //        queryTime, tf_cam_to_rgb_optical_frame);
        listener.waitForTransform(cam_frame_id_str, "camera_rgb_optical_frame",
                queryTime, ros::Duration(.5));
        listener.lookupTransform(cam_frame_id_str, "camera_rgb_optical_frame",
                queryTime, tf_cam_to_rgb_optical_frame);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Could not get calibration time before timeout! %s", ex.what());
                ros::Duration(0.01).sleep();
    }
    br.sendTransform(tf::StampedTransform(tf_cam_to_rgb_optical_frame,
            ros::Time::now(), "tf_cam", "calib_rgb_optical_pose"));
    if (DEBUG) {
        std::cout << "calib_result = " << tf_cam_to_rgb_optical_frame.getOrigin().x() << ", " <<
                tf_cam_to_rgb_optical_frame.getOrigin().y() << ", " << tf_cam_to_rgb_optical_frame.getOrigin().z() << std::endl;
    }
    tf::Quaternion calib_rot = tf_cam_to_rgb_optical_frame.getRotation();
    tf::Vector3 calib_translation = tf_cam_to_rgb_optical_frame.getOrigin();
    prior_tf = ar_calib_pose;
    prior_axis = calib_rot.getAxis();

    std::cout << "calib_tran = [" << calib_translation.getX() << " "
            << calib_translation.getY() << " "
            << calib_translation.getZ() << "] "
            << "calib_quat = ["
            << calib_rot.getX() << " "
            << calib_rot.getY() << " "
            << calib_rot.getZ() << " "
            << calib_rot.getW() << "]" << std::endl;
    if (logdata) {
        fos << calib_translation.getX() << " "
                << calib_translation.getY() << " "
                << calib_translation.getZ() << " "
                << calib_rot.getX() << " "
                << calib_rot.getY() << " "
                << calib_rot.getZ() << " "
                << calib_rot.getW() << std::endl;
    }
}

void CalibrateMocapAndCamera::setInitialTransform(tf::Transform nav_pose) {
    tf::Quaternion nav_frame = nav_pose.getRotation();
    tf::Matrix3x3 rotation_baselink2optical;
    rotation_baselink2optical.setRPY(-M_PI / 2, 0, -M_PI / 2);
    //        tf::Quaternion another_fin_variable;
    //        rotation_baselink2optical.getRotation(another_fin_variable);
    //        tf::Transform test(another_fin_variable, tf::Vector3(0,0,0));
    //        geometry_msgs::Transform test2;
    //        tf::transformTFToMsg(test,test2);
    //        std::cout << "test2 = " << test2 << std::endl;
    tf::Vector3 trans;
    trans = nav_pose.getOrigin();
    tf::Quaternion opt_frame;
    rotation_baselink2optical.getRotation(opt_frame);
    nav_frame *= opt_frame;
    tf::Transform opt_pose(nav_frame, trans);
    camera_pose = opt_pose;
}

void CalibrateMocapAndCamera::changePose(tf::Transform xform) {
    tf::Transform new_pose;
    //        tf::Quaternion delta_frame = xform.getRotation();
    //        tf::Vector3 delta_origin = xform.getOrigin();
    //        tf::Quaternion optical_frame = rgbd_pose.getRotation();
    //        tf::Vector3 optical_origin = rgbd_pose.getOrigin();
    //        optical_frame *= delta_frame;
    //        optical_origin += delta_origin;
    //        new_pose.setRotation(optical_frame);
    //        new_pose.setOrigin(optical_origin);
    new_pose.mult(camera_pose, xform);
    camera_pose = new_pose;

    //        geometry_msgs::Transform test2;
    //        tf::transformTFToMsg(test, test2);
    //        std::cout << "rgbd_pose = " << test2 << std::endl;

    br.sendTransform(tf::StampedTransform(camera_pose,
            ros::Time::now(), map_frame_id_str, rgb_frame_id_str));
}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "calibrate_mocap_and_camera");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    std::string tf_camera_marker_topic, tf_calib_marker_topic, ar_calib_topic;
    std::string optical_parent, optical_frame, _logfilename;
    bool _logdata;
    ros::NodeHandlePtr nodeptr(new ros::NodeHandle);
    ros::NodeHandle privnh("~");

    //    privnh.setParam("OpenCL_path", "/home.old/arwillis/NRC/ROS_RGBD_Odometry/ros_ws/src/rgbd_odometry/src/opencl");
    //    privnh.setParam("useOpenCL", false);

    privnh.param<std::string>("tf_cam_topic", tf_camera_marker_topic, "/tf_cam/pose");
    privnh.param<std::string>("tf_calib_topic", tf_calib_marker_topic, "/tf_calib/pose");
    privnh.param<std::string>("ar_calib_topic", ar_calib_topic, "/ar_single_board/transform");
    privnh.param("logdata", _logdata, false);
    privnh.param<std::string>("logfilename", _logfilename, "");
    privnh.param<std::string>("optical_parent", optical_parent, "optitrack");
    privnh.param<std::string>("optical_frame", optical_frame, "rgb_optical_frame");
    std::cout << "RGBD parent coordinate frame name = \"" << optical_parent << "\"" << std::endl;
    std::cout << "RGBD coordinate frame name =  \"" << optical_frame << "\"" << std::endl;
    CalibrateMocapAndCamera::Ptr engineptr(new CalibrateMocapAndCamera(optical_parent,
            optical_frame, _logdata, _logfilename));
    //    std::cout << "Initializing transform to ground truth from topic \""
    //            << tf_camera_marker_topic << "\"" << std::endl;
    engineptr->initializeSubscribers(nodeptr, tf_camera_marker_topic, tf_calib_marker_topic,
            ar_calib_topic);
    //    if (tf_truth_init_time > 0) {
    //        engineptr->setGroundTruthInitializationTime(tf_truth_init_time);
    //    }
    //std::cout << "Waiting for ground truth transform on topic \""
    //        << tf_camera_marker_topic << "\"..." << std::endl;
    //        tf::Transform identity;
    //        identity.setIdentity();
    //        engineptr->setInitialTransform(identity);

    engineptr->setTransformPublisher(nodeptr->advertise<geometry_msgs::Transform>("relative_xform", 1000));
    ros::spin();
    return 0;
}

