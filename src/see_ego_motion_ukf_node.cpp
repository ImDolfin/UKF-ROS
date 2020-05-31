#include "ros/ros.h"
#include "see_ego_motion_ukf/see_ego_motion_ukf_node.h"
#include "see_ego_motion_ukf/Vehicle.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <message_filters/subscriber.h>

// TODO: queue sizes müssen noch gesetzt werden = die der amis? Momentan überall 1
int main(int argc, char **argv) {
    ros::init(argc, argv, "see_ego_motion");

    see_ego_motion_ukf_node node;
    return 0;
}

/// Declaration and initialization of subscribers and publishers
/// Looping and rate limiting for publishers
see_ego_motion_ukf_node::see_ego_motion_ukf_node() {
    // subscriber and callback register
    ros::Subscriber odom;
    ros::Subscriber wheelSpeeds;
    ros::Subscriber slam;

    wheelSpeeds = nodeHandle.subscribe("/wheelspeeds", 1, &see_ego_motion_ukf_node::sensorWheelSpeedsCallback, this);
    odom = nodeHandle.subscribe("/odom", 1, &see_ego_motion_ukf_node::sensorOdomCallback, this);
    slam = nodeHandle.subscribe("/see_localization/vehicle_pose", 1, &see_ego_motion_ukf_node::slamPoseCallback, this);

    // broadcaster decalration and configuration
    tf2_ros::TransformBroadcaster transform_broadcaster;
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "/map";
    transform.child_frame_id = "/vehicle/ego_motion";

    gpsPoseArray.header.stamp = ros::Time::now();
    gpsPoseArray.header.frame_id = "/map";
    ukfPoseArray.header.stamp = ros::Time::now();
    ukfPoseArray.header.frame_id = "/map";
    unfilteredGPSPoseArray.header.stamp = ros::Time::now();
    unfilteredGPSPoseArray.header.frame_id = "/map";

    /// Publisher declration and initialization
    see_ego_motion_ukf::Vehicle msgToPub;
    static ros::Publisher pub = nodeHandle.advertise<see_ego_motion_ukf::Vehicle>("/see_ego_motion_ukf/vehicle", 1);
    static ros::Publisher pubGPSPose = nodeHandle.advertise<geometry_msgs::PoseArray>("/see_ego_motion_ukf/GpsPoses",
                                                                                      1);
    static ros::Publisher pubUKFPose = nodeHandle.advertise<geometry_msgs::PoseArray>("/see_ego_motion_ukf/UKFPoses",
                                                                                      1);
    static ros::Publisher pubUnfilteredGPSPose = nodeHandle.advertise<geometry_msgs::PoseArray>(
            "/see_ego_motion_ukf/unfilteredGPSPose", 1);

    /// rate sleep will wait in overall (see_ego will publish with 100hz later so this value is a placeholder currently)
    ros::Rate rate(100);

    /// while Node not killed/destructed/shutdowned
    while (ros::ok()) {
        /// Get state and covariance from sensorDataHandler
        Eigen::VectorXd state = sensorDataHandler.getFilteredState();
        Eigen::MatrixXd covariance = sensorDataHandler.getFilteredCovariance();

        ROS_INFO_STREAM(
                "x: " << state(0) << " y: " << state(1) << " yaw: " << state(2) << " yawRate: " << state(3) << " velo: "
                      << state(4) << " accel: " << state(5));

        transform.transform.translation.x = state(0);
        transform.transform.translation.y = state(1);
        transform.transform.translation.z = 0;
        tf2::Quaternion quat;

        /// set roll pitch yaw
        quat.setRPY(0, 0, state(2));
        tf2::convert(quat, transform.transform.rotation);

        /// publish tf message
        transform_broadcaster.sendTransform(transform);

        msgToPub.header.stamp = ros::Time::now();
        //msgToPub.isValid = true;
        msgToPub.yaw_rate = state(3);
        msgToPub.velocity = state(4);
        msgToPub.acceleration = state(5);
        msgToPub.x_std = covariance(0, 0);
        msgToPub.y_std = covariance(1, 1);
        msgToPub.yaw_std = covariance(2, 2);
        msgToPub.yaw_rate_std = covariance(3, 3);
        msgToPub.velocity_std = covariance(4, 4);
        msgToPub.acceleration_std = covariance(5, 5);

        geometry_msgs::PoseStamped pose;
        geometry_msgs::Quaternion msg;
        tf2::convert(sensorDataHandler.quaternion, msg);

        pose.pose.position.x = sensorDataHandler.X;
        pose.pose.position.y = sensorDataHandler.Y;
        pose.pose.position.z = 0;
        pose.pose.orientation = msg;
        gpsPoseArray.poses.push_back(pose.pose);

        geometry_msgs::Quaternion msgi;
        tf2::Quaternion quati;
        quati.setRPY(0, 0, state(2));
        tf2::convert(quati, msgi);
        geometry_msgs::PoseStamped posei;
        posei.pose.position.x = state(0);
        posei.pose.position.y = state(1);
        posei.pose.position.z = 0;
        posei.pose.orientation = msgi;
        ukfPoseArray.poses.push_back(posei.pose);

        geometry_msgs::PoseStamped posx;
        geometry_msgs::Quaternion msgx;
        tf2::convert(sensorDataHandler.quaternion, msgx);
        posx.pose.position.x = sensorDataHandler.uX;
        posx.pose.position.y = sensorDataHandler.uY;
        posx.pose.position.z = 0;
        posx.pose.orientation = msgx;
        unfilteredGPSPoseArray.poses.push_back(posx.pose);

        /// publish the message
        pub.publish(msgToPub);
        pubGPSPose.publish(gpsPoseArray);
        pubUKFPose.publish(ukfPoseArray);
        pubUnfilteredGPSPose.publish(unfilteredGPSPoseArray);

        /// Loops so the callbacks of the subscriber can be
        ros::spinOnce();

        /// sleeps the remaining time after execution to fullfill the 10ms/100hz of @rate
        rate.sleep();
    }
}

/// Destructor
see_ego_motion_ukf_node::~see_ego_motion_ukf_node() {

}

/// Callback operation for odometry call
/// Filters imu and gps data and calls the updateSensorState operation of the SenxsorDataHandler with imu and gps data
/// @param
void see_ego_motion_ukf_node::sensorOdomCallback(const gfr_common::Odom::ConstPtr &odom) {
    //ROS_INFO_STREAM("ODOM Callback" << endl);
    try { sensorDataHandler.updateSensorState(*odom); }
    catch (const std::exception &ex) { ROS_INFO_STREAM("Failed Odom"); }
}

/// Callback operation for whellspseed
/// calls the updateSensorState operation of the SensorDataHandler
/// @param
void see_ego_motion_ukf_node::sensorWheelSpeedsCallback(const gfr_common::WheelSpeeds::ConstPtr &wheelSpeeds) {
    //ROS_INFO_STREAM("Wheelspeeds Callback" << endl);
    try { sensorDataHandler.updateSensorState(*wheelSpeeds); }
    catch (const std::exception &ex) { ROS_INFO_STREAM("Failed WS"); }
}

/// Callback operation for slam
/// calls the updateSensorState operation of the SensorDataHandler
/// @param std_msgs::String slam
void see_ego_motion_ukf_node::slamPoseCallback(const std_msgs::String::ConstPtr &slam) {
    //ROS_INFO_STREAM("Slam_pose GPS Callback: " << slam->data.c_str() << endl);
    /*try { sensorDataHandler.updateSensorState(*slam); }
    catch (const std::exception &ex) { ROS_INFO_STREAM("Failed SLAM"); }*/
}
