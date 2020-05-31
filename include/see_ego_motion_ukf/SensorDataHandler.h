#ifndef SEE_EGO_MOTION_CPP_SENSORDATAHANDLER_H
#define SEE_EGO_MOTION_CPP_SENSORDATAHANDLER_H

#include"Eigen/Dense"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <map>
#include <gfr_common/Odom.h>
#include <gfr_common/WheelSpeeds.h>
#include "SensorFusionLogic.h"
#include "see_ego_motion_ukf/Sensors/SLAM.h"
#include "see_ego_motion_ukf/Sensors/GPS.h"
#include "see_ego_motion_ukf/Sensors/WheelSpeed.h"
#include "see_ego_motion_ukf/Sensors/IMU.h"
#include <memory>


#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;


/*!
 * This class contains the logic that handles input sensor values and the slam state
 */
class SensorDataHandler {

public:
    double X = 0;
    double Y = 0;
    double uX = 0;
    double uY = 0;
    tf2::Quaternion quaternion;

    /// SensorDataHandler constructor
    SensorDataHandler() = default;

    /// SensorDataHandler destructor
    ~SensorDataHandler() = default;

    /// updates the slam data and re-iterates the sensor fusion with the updates value
    /// \param slam message that contains the most recent slam update
    /// \return updated state vector
    VectorXd updateSensorState(string slam); //has to be changed to the actual ROS message input

    /// updates the GPS data and re-iterates the sensor fusion with the updates value
    /// \param gps message that contains the most recent GPS update
    /// \return updated state vector
    VectorXd updateSensorState(const sensor_msgs::NavSatFix &gps);

    /// updates the IMU data and re-iterates the sensor fusion with the updates value
    /// \param imu message that contains the most recent IMU update
    /// \return updated state vector
    VectorXd updateSensorState(const sensor_msgs::Imu &imu);

    /// updates the WheelSpeed data and re-iterates the sensor fusion with the updates value
    /// \param wheelSpeed message that contains the most recent Wheel Speed update
    /// \return updated state vector
    VectorXd updateSensorState(const gfr_common::WheelSpeeds &wheelSpeed);

    /// updates the Odometry data and re-iterates the sensor fusion with the updates value
    /// \param Odom message that contains the most recent GPS and IMU data
    /// \return updated state vector
    VectorXd updateSensorState(const gfr_common::Odom &odom);


    /// updates the synchronized gps and imu data and re-iterates the sensor fusion with the updates values
    /// \param gps message that contains the most recent GPS update
    /// \param imu message that contains the most recent IMU update
    /// \return updated state vector
    VectorXd updateSensorState(sensor_msgs::NavSatFix gps, sensor_msgs::Imu imu);

    /// getter for the filtered state vector
    /// \return VectorXd of filtered state vector
    VectorXd getFilteredState();

    /// getter for the filtered state covariance matrix
    /// \return MatrixXd of filtered state covariance matrix
    MatrixXd getFilteredCovariance();

private:
    /// SensorFusionLogic object which is responsible for the coordination of the Kalman Filter steps
    SensorFusionLogic sensorFusion;

    /// GPS object with {stdX, stdY)
    GPS gps{0.1, 0.1};
    /// filtered GPS object with {stdX, stdY}
    GPS filteredGPS{0.08, 0.08};
    /// IMU object with {stdAccel, stdYaw, stdYawRate}
    IMU imu{1, 0.2, 0.2};
    /// WheelSpeed object with{stdAccel}
    WheelSpeed wheelSpeed{0.5};
    ///Temporary SLAM placeholder Object
    SLAM slam{0.5, 0.5};

    /// retrieves the covariance matrix from the imu message
    /// \param imu message that contains the most recent IMU update
    /// \return retrieved covariance matrix
    MatrixXd retrieveCovariance(sensor_msgs::Imu imu);

    /// retrieves the measurements from the imu message
    /// \param imu message that contains the most recent IMU update
    /// \return retrieved measurement vector
    MatrixXd retrieveRawValues(sensor_msgs::Imu imu);

    /// retrieves the covariance matrix from the gps message
    /// \param gps message that contains the most recent GPS update
    /// \return retrieved covariance matrix
    MatrixXd retrieveCovariance(sensor_msgs::NavSatFix gps);

    /// retrieves the measurements from the gps message
    /// \param imu message that contains the most recent GPS update
    /// \return retrieved measurement vector
    MatrixXd retrieveRawValues(sensor_msgs::NavSatFix gps);

};


#endif //SEE_EGO_MOTION_CPP_SENSORDATAHANDLER_H
