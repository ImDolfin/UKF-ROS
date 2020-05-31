#ifndef SEE_EGO_MOTION_CPP_SENSORFUSIONLOGIC_H
#define SEE_EGO_MOTION_CPP_SENSORFUSIONLOGIC_H
#include "UnscentedKalmanFilter.h"
#include "Eigen/Dense"

#include "see_ego_motion_ukf/Sensors/GPS.h"
#include "see_ego_motion_ukf/Sensors/IMU.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/*!
 * The Sensor Fusion Logic is responsible for coordinating the input with the Kalman Filter
 */
class SensorFusionLogic {
  public:
    /// Constructor
    SensorFusionLogic();
    /// Destructor
    ~SensorFusionLogic() = default;

    //TODO: FOR LATER IMPLEMENT ROS CALLBACK

    /// Run predict and update  with new measured values - from a Sensor
    /// \param sensor class which contains its conversion logic and measurements
    /// \return VectorXd of the the updated state
    VectorXd fuseNewMeasurement(Sensor *sensor);

    /// Run predict and update  with new measured values - from multiple independent non overlapping Sensors
    /// \param firstSensor class which contains the sensors conversion logic and measurements
    /// \param secondSensor class which contains the sensors conversion logic and measurements
    /// \return VectorXd of the the updated state
    VectorXd fuseNewMeasurement(Sensor *firstSensor,  Sensor *secondSensor);

    /// Getter for the state vector
    /// \return VectorXd of the current UKFs state
    VectorXd getState();

    /// Getter for the states covariance matrix
    /// \return MatrixXd of the current UKFs state covariance
    MatrixXd getCovariance();


private:
    /// Old point in time
    ros::Time pointInTime;
    /// Unscented Kalman Filter Class
    UnscentedKalmanFilter UKalmanFilter;

    /// checks if the measurement Matrices overlap at one measurement position
    /// \param firstMeasurementMatrix measurementmatrix of the first sensor
    /// \param secondMeasurementMatrix measurementmatrix of the first sensor
    /// \return True if they overlap, false if not
    bool checkForMeasurementOverlaps(MatrixXd firstMeasurementMatrix, MatrixXd secondMeasurementMatrix);

    /// Fuse measurements from two sensors
    /// \param firstSensor class which contains the first sensors conversion logic and measurements
    /// \param secondSensor class which contains the second sensors conversion logic and measurements
    /// \return VectorXd of the fused measurements
    VectorXd fuseSensorMeasurements(Sensor *firstSensor,  Sensor *secondSensor);

    /// Fuse covariance matrices from multiple sensors
    /// \param firstSensor class which contains the first sensors conversion logic and measurements
    /// \param secondSensor class which contains the second sensors conversion logic and measurements
    /// \param indices indices of the combined measurements
    /// \return MatrixXd of the fused covariance
    MatrixXd fuseSensorCovariances(Sensor *firstSensor, Sensor *secondSensor, MatrixXd &indices);

    /// Fuse sigma point matrices from multiple sensors
    /// \param firstSensor class which contains the first sensors conversion logic and measurements
    /// \param secondSensor class which contains the second sensors conversion logic and measurements
    /// \param indices indices of the combined measurements
    /// \param predictedSigmaPoints matrix which contains the predicted sigma points
    /// \param measurementDimension dimension of the full measurement vector
    /// \return MatrixXd of the fused sigma point matrix
    MatrixXd fuseSensorSigmaPoints(Sensor *firstSensor, Sensor *secondSensor, MatrixXd &indices, const MatrixXd &predictedSigmaPoints, int measurementDimension);

    /// retrieves the indices of the measurements state positions in relation to
    /// the state positions and and the sensor which contains the mapped measurement
    /// \param firstMeasurementMatrix measurement matrix of the first sensor
    /// \param secondMeasurementMatrix measurement matrix of the second sensor
    /// \param measurementDimension measurement dimension
    /// \return MatrixXd of indices on col(0) -> sensor(1 = firstSensor, 2 = secondSensor),
    ///                             on col(1) -> statePosition (between 0 and kalmanfilters stateDimension)
    MatrixXd getMeasurementIndices(MatrixXd firstMeasurementMatrix, MatrixXd secondMeasurementMatrix, int measurementDimension);

    /// Perform the prediction step of the kalman filter
    /// \param newTime time of the measurement
    /// \return MatrixXd of the predicted sigma points
    MatrixXd runKalmanFilterPrediction(ros::Time newTime);

    /// Performs the update step of the kalman filter
    /// \param measuredState VectorXd containing the original measurements
    /// \param measurementNoise MatrixXd containing the noise matrix of the Sensor
    /// \param measurementSigmaPoints MatrixXd containing the sigma points from the measurements
    /// \param predictedSigmaPoints MatrixXd containing the predicted sigma points
    /// \param measurementDimension int of the measurement vector dimension
    /// \param indexOfMeasuredYaw int of the index of the measured vectors yaw
    ///                          (use any value below 0 or above 5 to indicate that there is no yaw)
    /// \return VectorXd of the updated state
    VectorXd runKalmanFilterUpdate(VectorXd &measuredState, MatrixXd &measurementNoise,
            MatrixXd &measurementSigmaPoints, MatrixXd &predictedSigmaPoints,
            const int &measurementDimension, const int &indexOfMeasuredYaw);
};


#endif //SEE_EGO_MOTION_CPP_SENSORFUSIONLOGIC_H
