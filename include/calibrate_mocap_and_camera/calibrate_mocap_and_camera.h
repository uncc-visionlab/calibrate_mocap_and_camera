/* 
 * File:   Feature3DEngine.h
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */

#ifndef CALIBRATE_MOCAP_AND_CAMERA_H
#define	CALIBRATE_MOCAP_AND_CAMERA_H

// Standard C++ includes
#include <string>
#include <iostream>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class CalibrateMocapAndCamera {
public:
    typedef boost::shared_ptr<CalibrateMocapAndCamera> Ptr;

    CalibrateMocapAndCamera(std::string optical_parent, std::string optical_frame) :
    map_frame_id_str(optical_parent),
    rgbd_frame_id_str(optical_frame) {
    };

    virtual ~CalibrateMocapAndCamera() {
    };

    void initializeGroundTruthSubscriber(ros::NodeHandlePtr nodeptr,
            std::string topic, int timeval = 10) {
        sub_tf_truth = nodeptr->subscribe(topic, 10, &CalibrateMocapAndCamera::tf_truth_Callback, this);
        setGroundTruthInitializationTime(timeval);
    }

    void setGroundTruthInitializationTime(int timeval) {
        tf_truth_init_time = timeval;
    }

    void setInitialTransform(tf::Transform nav_pose);

    void tf_truth_Callback(const geometry_msgs::TransformStampedConstPtr& tf_truth);

    void setTransformPublisher(ros::Publisher p) {
        pubXforms = p;
    }

    void publishTransform(geometry_msgs::Transform xform) {
        pubXforms.publish(xform);
    }

    void changePose(tf::Transform xform);

    void broadcastTFMsg(tf::Transform xform, std::string parent_frame_id,
            std::string child_frame_id) {
        br.sendTransform(tf::StampedTransform(xform,
                ros::Time::now(), parent_frame_id, child_frame_id));
    }

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    CalibrateMocapAndCamera(const CalibrateMocapAndCamera& yRef);
    CalibrateMocapAndCamera& operator=(const CalibrateMocapAndCamera& yRef);

    ros::Publisher pubXforms;
    ros::Publisher pubOdomMsg;

    // variable used when using a ground truth reference initialization
    int tf_truth_init_time;
    ros::Subscriber sub_tf_truth;

    std::string map_frame_id_str;
    std::string rgbd_frame_id_str;

    tf::TransformBroadcaster br;

    tf::Transform camera_pose;
    tf::Transform calibration_board_marker_pose;
    tf::Transform camera_marker_pose;
};
#endif	/* CALIBRATE_MOCAP_AND_CAMERA_H */

