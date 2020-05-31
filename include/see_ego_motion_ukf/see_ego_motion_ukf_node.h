#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <gfr_common/Odom.h>
#include <gfr_common/WheelSpeeds.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "see_ego_motion_ukf/SensorDataHandler.h"

class see_ego_motion_ukf_node {
public:
    /// constructor and destructor
    see_ego_motion_ukf_node();

    ~see_ego_motion_ukf_node();

    /// Callback Operations for Subscriber
    void sensorGpsImuCallback(const sensor_msgs::NavSatFix::ConstPtr &gps, const sensor_msgs::Imu::ConstPtr &imu);

    void sensorOdomCallback(const gfr_common::Odom::ConstPtr &odom);

    void sensorWheelSpeedsCallback(const gfr_common::WheelSpeeds::ConstPtr &wheel_speeds);

    void slamPoseCallback(const std_msgs::String::ConstPtr &slam);

private:
    /// Ros Nodehandle needed for subscribers and publishers
    ros::NodeHandle nodeHandle;

    // object of sensorDataHandler to interact with sensors (update values)
    SensorDataHandler sensorDataHandler;

    geometry_msgs::PoseArray gpsPoseArray;
    geometry_msgs::PoseArray ukfPoseArray;
    geometry_msgs::PoseArray unfilteredGPSPoseArray;
};
