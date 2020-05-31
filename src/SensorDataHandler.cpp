#include "see_ego_motion_ukf/SensorDataHandler.h"
#include"Eigen/Dense"
#include "ros/ros.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/// updates the Odometry data and re-iterates the sensor fusion with the updates value
/// \param Odom message that contains the most recent GPS and IMU data
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(const gfr_common::Odom &odom) {

    VectorXd gpsVector(3);
    gpsVector(0) = odom.gps_position.x;
    gpsVector(1) = odom.gps_position.y;
    gpsVector(2) = odom.gps_position.z;//pose.position.z;

    VectorXd gpsFilteredVector(3);
    gpsFilteredVector(0) = odom.pose.position.x;
    gpsFilteredVector(1) = odom.pose.position.y;
    gpsFilteredVector(2) = odom.pose.position.z;

    MatrixXd gpsCovariance(3, 3);
    gpsCovariance.fill(0.0);

    VectorXd imuVector(8);
    imuVector(0) = odom.imu_acceleration.x;//twist.linear.x; // acceleration at x axis
    imuVector(1) = odom.imu_acceleration.y; // acceleration at y axis
    imuVector(2) = odom.imu_acceleration.z; // acceleration at z axis
    imuVector(3) = odom.imu_angular_rate.z; //twist.angular.z; // yaw rate - angular velocity
    imuVector(4) = odom.pose.orientation.w;
    imuVector(5) = odom.pose.orientation.x;
    imuVector(6) = odom.pose.orientation.y;
    imuVector(7) = odom.pose.orientation.z;
    MatrixXd imuCovariance(3, 3);
    imuCovariance.fill(0.0);

    this->filteredGPS.setRawInput(gpsFilteredVector, gpsCovariance, odom.header.stamp);
    double Yaw = tf2::getYaw(odom.pose.orientation);
    this->quaternion.setRPY(0, 0, Yaw);
    Eigen::VectorXd Meas = this->filteredGPS.getMeasurements();
    this->X = Meas(0);
    this->Y = Meas(1);

    this->gps.setRawInput(gpsVector, gpsCovariance, odom.header.stamp);
    Meas = this->gps.getMeasurements();
    this->uX = Meas(0);
    this->uY = Meas(1);
    ROS_WARN_STREAM("lat: " << gpsVector(0) << " lon: " << gpsVector(1) << " alt: " << gpsVector(2));
    this->imu.setRawInput(imuVector, imuCovariance, odom.header.stamp);
    ROS_WARN_STREAM("yaw: " << Yaw << " yawRate: " << imuVector(3) << " accelX: " << imuVector(0) << " accelY: "
                            << imuVector(1));
    sensorFusion.fuseNewMeasurement(&this->gps, &this->imu);

    return sensorFusion.fuseNewMeasurement(&this->filteredGPS);
}

/// updates the WheelSpeed data and re-iterates the sensor fusion with the updates value
/// \param wheelSpeed message that contains the most recent Wheel Speed update
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(const gfr_common::WheelSpeeds &ws) {
    VectorXd wheelSpeedVector(4);

    wheelSpeedVector(0) = ws.front_left;
    wheelSpeedVector(1) = ws.front_right;
    wheelSpeedVector(2) = ws.rear_left;
    wheelSpeedVector(3) = ws.rear_right;
    MatrixXd covariance(1, 1);
    covariance.fill(0.0);
    this->wheelSpeed.setRawInput(wheelSpeedVector, covariance, ws.header.stamp);
    return sensorFusion.fuseNewMeasurement(&this->wheelSpeed);

}

/// getter for the filtered state vector
/// \return VectorXd of filtered state vector
VectorXd SensorDataHandler::getFilteredState() {
    return this->sensorFusion.getState();
}

/// getter for the filtered state covariance matrix
/// \return MatrixXd of filtered state covariance matrix
MatrixXd SensorDataHandler::getFilteredCovariance() {
    return this->sensorFusion.getCovariance();
}




/******
 ****** THE FOLLOWING METHOD DECLARATIONS ARE NOT USED BUT MEANT TO BE AN ORIENTATION FOR FUTURE IMPLEMENTATIONS.
 ****** THEREFORE, WE DID NOT TAKE THEM OUT SO NEW GENERATIONS CAN EASIER UNDERSTAND WHAT HAS TO BE DONE FOR THE
 ****** DIFFERENT KINDS OF SENSORS AND HOW TO PROCEED FROM THERE.
 ******
 */

/// updates the slam data and re-iterates the sensor fusion with the updates value
/// \param slam message that contains the most recent slam update
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(string slam) {//remove placeholder for actual slam message
    return this->sensorFusion.getState();
}

/// updates the GPS data and re-iterates the sensor fusion with the updates value
/// \param gps message that contains the most recent GPS update
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(const sensor_msgs::NavSatFix &gps) {
    VectorXd gpsVector = retrieveRawValues(gps);
    MatrixXd gpsCovariance = retrieveCovariance(gps);
    //this->sensorDictionary["GPS"].setRawInput(gpsVector, gpsCovariance , gps.header.stamp);
    this->gps.setRawInput(gpsVector, gpsCovariance, gps.header.stamp);
    //return sensorFusion.fuseNewMeasurement(&this->sensorDictionary["GPS"], true);
    return this->sensorFusion.fuseNewMeasurement(&this->gps);
}

/// updates the IMU data and re-iterates the sensor fusion with the updates value
/// \param imu message that contains the most recent IMU update
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(const sensor_msgs::Imu &imu) {
    VectorXd imuVector = retrieveRawValues(imu);
    MatrixXd imuCovariance = retrieveCovariance(imu);
    //this->sensorDictionary["IMU"].setRawInput(imuVector, imuCovariance, imu.header.stamp);
    this->imu.setRawInput(imuVector, imuCovariance, imu.header.stamp);
    //return sensorFusion.fuseNewMeasurement(&this->sensorDictionary["IMU"], true);
    return this->sensorFusion.fuseNewMeasurement(&this->imu);
}



/// updates the synchronized gps and imu data and re-iterates the sensor fusion with the updates values
/// \param gps message that contains the most recent GPS update
/// \param imu message that contains the most recent IMU update
/// \return updated state vector
VectorXd SensorDataHandler::updateSensorState(sensor_msgs::NavSatFix gps, sensor_msgs::Imu imu) {
    //ROS_INFO_STREAM(std::to_string(frequency));
    VectorXd imuVector = retrieveRawValues(imu);
    MatrixXd imuCovariance = retrieveCovariance(imu);
    //this->sensorDictionary["IMU"].setRawInput(imuVector, imuCovariance, imu.header.stamp);
    this->imu.setRawInput(imuVector, imuCovariance, imu.header.stamp);

    VectorXd gpsVector = retrieveRawValues(gps);
    MatrixXd gpsCovariance = retrieveCovariance(gps);
    //this->sensorDictionary["GPS"].setRawInput(gpsVector, gpsCovariance , gps.header.stamp);
    this->gps.setRawInput(gpsVector, gpsCovariance, gps.header.stamp);

    return sensorFusion.fuseNewMeasurement(&this->gps, &this->imu);
}

/// retrieves the covariance matrix from the imu message
/// \param imu message that contains the most recent IMU update
/// \return retrieved covariance matrix
MatrixXd SensorDataHandler::retrieveCovariance(sensor_msgs::Imu imu) {
    MatrixXd imuCovariance(3, 3);
    imuCovariance.fill(0.0);
    // yaw covariance at axes z being the last diagonal element
    imuCovariance(0, 0) = imu.orientation_covariance[8];
    // yaw rate covariance at axes z being the last diagonal element
    imuCovariance(1, 1) = imu.angular_velocity_covariance[8];
    //average the acceleration covariance of all axis
    imuCovariance(2, 2) = (imu.linear_acceleration_covariance[0] +
                           imu.linear_acceleration_covariance[4] +
                           imu.linear_acceleration_covariance[8]) / 3;

    return imuCovariance;
}

/// retrieves the measurements from the imu message
/// \param imu message that contains the most recent IMU update
/// \return retrieved measurement vector
MatrixXd SensorDataHandler::retrieveRawValues(sensor_msgs::Imu imu) {
    VectorXd imuVector(8);
    imuVector(0) = imu.linear_acceleration.x; // acceleration at x axis
    imuVector(1) = imu.linear_acceleration.y; // acceleration at y axis
    imuVector(2) = imu.linear_acceleration.z; // acceleration at z axis
    imuVector(3) = imu.angular_velocity.z; // yaw rate - angular velocity
    imuVector(4) = imu.orientation.w;
    imuVector(5) = imu.orientation.x;
    imuVector(6) = imu.orientation.y;
    imuVector(7) = imu.orientation.z;

    return imuVector;
}

/// retrieves the covariance matrix from the gps message
/// \param gps message that contains the most recent GPS update
/// \return retrieved covariance matrix
MatrixXd SensorDataHandler::retrieveCovariance(sensor_msgs::NavSatFix gps) {
    MatrixXd gpsCovariance(2, 2);
    gpsCovariance.fill(0.0);
    gpsCovariance(0, 0) = gps.position_covariance[0];
    gpsCovariance(1, 1) = gps.position_covariance[4];
    return gpsCovariance;
}

/// retrieves the measurements from the gps message
/// \param imu message that contains the most recent GPS update
/// \return retrieved measurement vector
MatrixXd SensorDataHandler::retrieveRawValues(sensor_msgs::NavSatFix gps) {
    VectorXd gpsVector(3);
    gpsVector(0) = gps.latitude;
    gpsVector(1) = gps.longitude;
    gpsVector(2) = gps.altitude;
    return gpsVector;
}
