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

void CalibrateMocapAndCamera::tf_truth_Callback(const geometry_msgs::TransformStampedConstPtr& tf_truth) {
    static geometry_msgs::TransformStampedConstPtr first_tf;
    static int num_msgs;
    static tf::Transform initialTransform;
    static Eigen::Vector3f eig_trns;
    static Eigen::Vector4f eig_quat;
    if (!first_tf) {
        first_tf = tf_truth;
        num_msgs = 0;
        initialTransform.setIdentity();
        eig_quat[0] = eig_quat[1] = eig_quat[2] = eig_quat[3] = 0;
        eig_trns[0] = eig_trns[1] = eig_trns[2] = 0;
    }
    geometry_msgs::Vector3 translation;
    geometry_msgs::Quaternion quaternion;
    translation = tf_truth->transform.translation;
    quaternion = tf_truth->transform.rotation;
    eig_trns[0] += translation.x;
    eig_trns[1] += translation.y;
    eig_trns[2] += translation.z;
    eig_quat[0] += quaternion.x;
    eig_quat[1] += quaternion.y;
    eig_quat[2] += quaternion.z;
    eig_quat[3] += quaternion.w;
    num_msgs++;
    ROS_DEBUG("Heard tf_truth.");
    //    std::cout << "rotation = " << tf_truth->transform.rotation << std::endl
    //            << "translation = " << tf_truth->transform.translation << std::endl;
    if (tf_truth->header.stamp.sec - first_tf->header.stamp.sec > tf_truth_init_time) {
        std::cout << "Initialization time has expired." << std::endl;
        eig_trns[0] /= num_msgs;
        eig_trns[1] /= num_msgs;
        eig_trns[2] /= num_msgs;
        eig_quat[0] /= num_msgs;
        eig_quat[1] /= num_msgs;
        eig_quat[2] /= num_msgs;
        eig_quat[3] /= num_msgs;
        tf::Vector3 tval(eig_trns[0], eig_trns[1], eig_trns[2]);
        tf::Quaternion qval(eig_quat[0], eig_quat[1], eig_quat[2], eig_quat[3]);
        qval = qval.normalize();
        tf::Transform initialTransform(qval, tval);
        std::cout << "Received " << num_msgs << " values for ground truth."
                << " Setting ground truth transform to: " << std::endl
                << "rotation = (" << qval.getX() << ", " << qval.getY()
                << ", " << qval.getZ() << ", " << qval.getW() << ")" << std::endl
                << "translation = (" << tval.getX() << ", " << tval.getY()
                << ", " << tval.getZ() << ")" << std::endl;
        setInitialTransform(initialTransform);
        sub_tf_truth.shutdown();
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
            ros::Time::now(), map_frame_id_str, rgbd_frame_id_str));
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
    std::string tf_camera_marker_topic, tf_calib_marker_topic;
    std::string optical_parent, optical_frame;
    bool tf_truth_initialize;
    int tf_truth_init_time;
    ros::NodeHandlePtr nodeptr(new ros::NodeHandle);
    ros::NodeHandle privnh("~");

    //    privnh.setParam("OpenCL_path", "/home.old/arwillis/NRC/ROS_RGBD_Odometry/ros_ws/src/rgbd_odometry/src/opencl");
    //    privnh.setParam("useOpenCL", false);

    privnh.param("tf_truth_initialize", tf_truth_initialize, false);
    privnh.param<std::string>("tf_cam_topic", tf_camera_marker_topic, "");
    privnh.param("tf_truth_init_time", tf_truth_init_time, -1);

    privnh.param<std::string>("optical_parent", optical_parent, "optitrak");
    privnh.param<std::string>("optical_frame", optical_frame, "rgbd_frame");
    std::cout << "RGBD parent coordinate frame name = \"" << optical_parent << "\"" << std::endl;
    std::cout << "RGBD coordinate frame name =  \"" << optical_frame << "\"" << std::endl;
    CalibrateMocapAndCamera::Ptr engineptr(new CalibrateMocapAndCamera(optical_parent, optical_frame));

    if (tf_truth_initialize) {
        std::cout << "Initializing transform to ground truth from topic \""
                << tf_camera_marker_topic << "\" wait " << tf_truth_init_time
                << " seconds." << std::endl;
        engineptr->initializeGroundTruthSubscriber(nodeptr, tf_camera_marker_topic);
        if (tf_truth_init_time > 0) {
            engineptr->setGroundTruthInitializationTime(tf_truth_init_time);
        }
        std::cout << "Waiting for ground truth transform on topic \""
                << tf_camera_marker_topic << "\"..." << std::endl;
    } else {
        tf::Transform identity;
        identity.setIdentity();
        engineptr->setInitialTransform(identity);
    }

    engineptr->setTransformPublisher(nodeptr->advertise<geometry_msgs::Transform>("relative_xform", 1000));
    ros::spin();
    return 0;
}

