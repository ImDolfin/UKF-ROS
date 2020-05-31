#include "see_ego_motion_ukf/SensorFusionLogic.h"
#include <Eigen/Dense>
#include <ros/ros.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Constructor
SensorFusionLogic::SensorFusionLogic() {
    this->pointInTime = ros::Time::now();
}

/// Run predict and update  with new measured values - from a Sensor
/// \param sensor class which contains its conversion logic and measurements
/// \param isUpdateOnly boolean indicating if the sensor is only updating e.g. the time delta will be 0
/// \return VectorXd of the the updated state
VectorXd SensorFusionLogic::fuseNewMeasurement(Sensor *sensor) {
    //! Get the converted measurement from the sensor
    VectorXd measuredState = sensor->getMeasurements();
    ros::Time newTime = sensor->getTimeStamp();
    if (!UKalmanFilter.doesVectorContainValues(measuredState)) {
        throw std::invalid_argument("The provided measurements vector doesn't contain measurements!");
    }
    MatrixXd measurementMatrix = sensor->getMeasurementMatrix();
    if (!UKalmanFilter.doesMatrixContainValues(measurementMatrix) || measurementMatrix.rows() != sensor->getMeasurementDimension()) {
        throw std::invalid_argument("The provided measurement matrix is illegal!");
    }
    //! Check if the kalman filter is initialized, if it isn't then initialize it with the measurement
    if (!UKalmanFilter.isInitialized()) {
        UKalmanFilter.initialize(measuredState);
        this->pointInTime = newTime;
        return measuredState;
    }
    ROS_INFO_STREAM("Measurements: ");
    ROS_INFO_STREAM(measuredState.transpose());

    //! Predict step of the Kalman filter
    MatrixXd predictedSigmaPoints;
    predictedSigmaPoints = runKalmanFilterPrediction(newTime);

    //! Retrieve the dimension of the measurement state vector
    const int measurementDimension = sensor->getMeasurementDimension();
    //ROS_INFO("Sensor: %i", sensor->getMeasurementDimension());

    //! Transform the predicted sigma points the the measurement dimension
    MatrixXd measurementSigmaPoints = sensor->transformPointsToMeasurementSpace(predictedSigmaPoints);

    int yawPosition = 1000;
    for (int i = 0; i < measurementDimension; i++) {
        if (measurementMatrix(i, 2)) {
            yawPosition = i;
        }
    }
    //! Boil the measured state, which contains 0's for not measured values, down to the relevant values
    VectorXd predictedState = measurementMatrix * measuredState;

    measuredState = predictedState;

    //! Update step of the Kalman filter
    //! Uses the covariance as Noise matrix -> if covariance is "illegal", then the sensor will return the noise
    MatrixXd measurementCovariance = sensor->getMeasurementNoise();
    return runKalmanFilterUpdate(measuredState, measurementCovariance, measurementSigmaPoints, predictedSigmaPoints,
                                measurementDimension, yawPosition);

}

/// Run predict and update  with new measured values - from multiple independent non overlapping Sensors
/// \param firstSensor class which contains the sensors conversion logic and measurements
/// \param secondSensor class which contains the sensors conversion logic and measurements
/// \return VectorXd of the the updated state
VectorXd SensorFusionLogic::fuseNewMeasurement(Sensor *firstSensor, Sensor *secondSensor) {
    MatrixXd firstMeasurementMatrix, secondMeasurementMatrix;

    firstMeasurementMatrix = firstSensor->getMeasurementMatrix();
    secondMeasurementMatrix = secondSensor->getMeasurementMatrix();
    if ((!UKalmanFilter.doesMatrixContainValues(firstMeasurementMatrix) || firstMeasurementMatrix.rows() != firstSensor->getMeasurementDimension())
    || (!UKalmanFilter.doesMatrixContainValues(secondMeasurementMatrix) || secondMeasurementMatrix.rows() != secondSensor->getMeasurementDimension()))
    {
        throw std::invalid_argument("The provided measurement matrix is illegal!");
    }

    //! First check if there are any values which overlap between the sensors for example -> both measure acceleration
    //! This is not implemented and will therefore lead to failures. This means, that throwing an Exception will stop this illegal atempt
    bool overlaps = checkForMeasurementOverlaps(firstMeasurementMatrix, secondMeasurementMatrix);
    if (overlaps)
        throw std::invalid_argument("The Measurements overlap, that is not allowed in the context of this method");

    //! Get the converted measurement from the sensor
    VectorXd measuredState = fuseSensorMeasurements(firstSensor, secondSensor);
    ros::Time newTime = firstSensor->getTimeStamp();

    if (!UKalmanFilter.doesVectorContainValues(measuredState)) {
        throw std::invalid_argument("The provided measurements vector doesn't contain measurements!");
    }

    ROS_INFO_STREAM("Measurements: ");
    ROS_INFO_STREAM(measuredState.transpose());

    //! Check if the kalman filter is initialized, if it isn't then initialize it with the measurement
    if (!UKalmanFilter.isInitialized()) {
        UKalmanFilter.initialize(measuredState);
        this->pointInTime = newTime;
        return measuredState;
    }

    //! Do the kalman filters prediction step with the delta time;
    MatrixXd predictedSigmaPoints = runKalmanFilterPrediction(newTime);

    //! Retrieve the dimension of the measurement state vector
    const int measurementDimension = firstSensor->getMeasurementDimension() + secondSensor->getMeasurementDimension();
    VectorXd measurements(measurementDimension);
    measurements.fill(0.0);

    //! Find the state indices of the measurements and their corresponding sensor
    MatrixXd indices = getMeasurementIndices(firstMeasurementMatrix, secondMeasurementMatrix, measurementDimension);

    //! Reassign the full measurement state to the measurements state of size ->measurement dimension, using the indices
    for (int i = 0; i < measurementDimension; i++) {
        measurements(i) = measuredState(indices(i, 1));
    }

    //! Retrieve the fused measurement sigma point matrix using the indices
    MatrixXd measurementSigmaPoints(measurementDimension, predictedSigmaPoints.cols());
    measurementSigmaPoints = fuseSensorSigmaPoints(firstSensor, secondSensor, indices, predictedSigmaPoints,
                                                   measurementDimension);

    //! Retrieve the fused measurement covariance matrix using the indices
    MatrixXd measurementCovariance = fuseSensorCovariances(firstSensor, secondSensor, indices);

    //! Find the yaw position if there is any
    int yawPosition = 1000;
    for (int i = 0; i < measurementDimension; i++) {
        if (indices(i, 1) == 2) {
            yawPosition = i;
        }
    }

    //! Update step of the Kalmanfilter
    return runKalmanFilterUpdate(measurements, measurementCovariance, measurementSigmaPoints, predictedSigmaPoints,
                                 measurementDimension, yawPosition);

}

/// retrieves the indices of the measurements state positions in relation to
/// the state positions and and the sensor which contains the mapped measurement
/// \param firstMeasurementMatrix measurement matrix of the first sensor
/// \param secondMeasurementMatrix measurement matrix of the second sensor
/// \param measurementDimension measurement dimension
/// \return MatrixXd of indices on col(0) -> sensor(1 = firstSensor, 2 = secondSensor),
///                             on col(1) -> statePosition (between 0 and kalmanfilters stateDimension)
MatrixXd SensorFusionLogic::getMeasurementIndices(MatrixXd firstMeasurementMatrix, MatrixXd secondMeasurementMatrix,
                                                  int measurementDimension) {
    //! Declare the index vectors
    VectorXd firstIndices(firstMeasurementMatrix.rows());
    VectorXd secondIndices(secondMeasurementMatrix.rows());
    MatrixXd indices(measurementDimension, 2);

    //! Assign the indices of the measurement in relation to the corresponding state vector index
    //! e.g. sensor measures (X, Y, Accel), state has (X, Y, Yaw, YawRate, Velocity, Acceleration)
    //! so the resulting array of indices maps the measured positions to the states
    //! result is -> (0, 1, 5)
    for (int i = 0; i < firstMeasurementMatrix.rows(); i++) {
        for (int j = 0; j < firstMeasurementMatrix.cols(); j++) {
            if (firstMeasurementMatrix(i, j) == 1)
                firstIndices(i) = j;
        }
    }
    //! the same procedure as above applies to the second sensor
    for (int i = 0; i < secondMeasurementMatrix.rows(); i++) {
        for (int j = 0; j < secondMeasurementMatrix.cols(); j++) {
            if (secondMeasurementMatrix(i, j) == 1)
                secondIndices(i) = j;
        }
    }

    //! Now sort the two index vectors together based on the state index and who measures the value
    //! so example: sensor one indices (0, 1, 5), sensor two indices (2, 3)
    //! now sort those values, sensor one will be represented by 1 and two by 2
    //! so the result is a matrix of (measurementeDimension * 2)
    //!         --> column index 1 shows the state index        [0, 1, 2, 3, 5]
    //!         --> column index 0 shows the responsible sensor [1, 1, 2, 2, 1]
    //! the following loop only works because the state is ordered and not overlaped
    int lowestStateIndex = -1;
    // current measurement index stands for the index value of the state which is currently the lowest of the two sensors
    int currentMeasurementIndex = 5;
    // decider is the variable which is stands for the sensor that is responsible for the current measurement index
    int decider = 0;
    for (int i = 0; i < measurementDimension; i++) {
        // find lowest state index between lowest and current
        for (int j = 0; j < firstIndices.size(); j++) {
            if (firstIndices(j) < currentMeasurementIndex && firstIndices(j) > lowestStateIndex) {
                currentMeasurementIndex = firstIndices(j);
                decider = 1;
            }
        }
        // do the same for the second sensor indices
        for (int j = 0; j < firstIndices.size(); j++) {
            if (secondIndices(j) < currentMeasurementIndex && secondIndices(j) > lowestStateIndex) {
                currentMeasurementIndex = secondIndices(j);
                decider = 2;
            }
        }
        //reassign and up the lowest state index to the current measurement
        lowestStateIndex = currentMeasurementIndex;
        indices(i, 0) = decider;
        indices(i, 1) = currentMeasurementIndex;
        currentMeasurementIndex = 5;
    }
    return indices;
}

/// checks if the measurement Matrices overlap at one measurement position
/// e.g. if two sensors measure the same variable -> yaw for example
/// so if this fails, either the measurement matrix is wrong or the measurements overlap
/// \param firstMeasurementMatrix measurementmatrix of the first sensor
/// \param secondMeasurementMatrix measurementmatrix of the first sensor
/// \return True if they overlap, false if not
bool SensorFusionLogic::checkForMeasurementOverlaps(MatrixXd firstMeasurementMatrix, MatrixXd secondMeasurementMatrix) {
    for (int i = 0; i < firstMeasurementMatrix.rows(); i++) {
        for (int j = 0; j < secondMeasurementMatrix.rows(); j++) {
            if (firstMeasurementMatrix.row(i) == secondMeasurementMatrix.row(j))
                return true;
        }
    }
    return false;
}

/// Fuse covariance matrices from multiple sensors
/// \param firstSensor class which contains the first sensors conversion logic and measurements
/// \param secondSensor class which contains the second sensors conversion logic and measurements
/// \param indices indices of the combined measurements
/// \return MatrixXd of the fused covariance
MatrixXd SensorFusionLogic::fuseSensorCovariances(Sensor *firstSensor, Sensor *secondSensor, MatrixXd &indices) {
    int firstIndexCounter = 0;
    int secondIndexCounter = 0;
    int measurementDimension = indices.rows();
    MatrixXd firstCovariance = firstSensor->getMeasurementCovariance();
    MatrixXd secondCovariance = secondSensor->getMeasurementCovariance();

    MatrixXd measurementCovariance(measurementDimension, measurementDimension);
    measurementCovariance.fill(0.0);

    //! based on the indices one can skip through the matrices because each ordered state index
    //! has its corresponding sensor attached to it
    for (int i = 0; i < measurementDimension; i++) {
        if (indices(i, 0) == 1) {
            measurementCovariance(i, i) = firstCovariance(firstIndexCounter, firstIndexCounter);
            firstIndexCounter++;
        }
        if (indices(i, 0) == 2) {
            measurementCovariance(i, i) = secondCovariance(secondIndexCounter, secondIndexCounter);
            secondIndexCounter++;
        }
    }
    return measurementCovariance;
}

/// Fuse sigma point matrices from multiple sensors
/// \param firstSensor class which contains the first sensors conversion logic and measurements
/// \param secondSensor class which contains the second sensors conversion logic and measurements
/// \param indices indices of the combined measurements
/// \param predictedSigmaPoints matrix which contains the predicted sigma points
/// \param measurementDimension dimension of the full measurement vector
/// \return MatrixXd of the fused sigma point matrix
MatrixXd SensorFusionLogic::fuseSensorSigmaPoints(Sensor *firstSensor, Sensor *secondSensor, MatrixXd &indices,
                                                  const MatrixXd &predictedSigmaPoints, int measurementDimension) {
    //! Transform the predicted sigma points the the measurement dimension
    MatrixXd firstMeasurementSigmaPoints = firstSensor->transformPointsToMeasurementSpace(predictedSigmaPoints);
    //! Transform the predicted sigma points the the measurement dimension
    MatrixXd secondMeasurementSigmaPoints = secondSensor->transformPointsToMeasurementSpace(predictedSigmaPoints);

    MatrixXd measurementSigmaPoints(measurementDimension, secondMeasurementSigmaPoints.cols());

    int firstIndexCounter = 0;
    int secondIndexCounter = 0;

    //! based on the indices one can skip through the matrices because each ordered state index
    //! has its corresponding sensor attached to it
    for (int i = 0; i < measurementDimension; i++) {
        if (indices(i, 0) == 1) {
            measurementSigmaPoints.row(i) = firstMeasurementSigmaPoints.row(firstIndexCounter);
            firstIndexCounter++;
        }
        if (indices(i, 0) == 2) {
            measurementSigmaPoints.row(i) = secondMeasurementSigmaPoints.row(secondIndexCounter);
            secondIndexCounter++;
        }
    }

    return measurementSigmaPoints;
}

/// Fuse measurements from two sensors
/// \param firstSensor class which contains the first sensors conversion logic and measurements
/// \param secondSensor class which contains the second sensors conversion logic and measurements
/// \return VectorXd of the fused measurements
VectorXd SensorFusionLogic::fuseSensorMeasurements(Sensor *firstSensor, Sensor *secondSensor) {
    VectorXd firstSensorVector = firstSensor->getMeasurements();
    VectorXd secondSensorVector = secondSensor->getMeasurements();

    VectorXd measurements(6);
    measurements.fill(0.0);
    //! because overlapping measurements are permitted, one can simple add up the vectors
    measurements += firstSensorVector + secondSensorVector; //values are 0 where there aren't any calculated
    return measurements;
}

/// Getter for the state vector
/// \return VectorXd of the current UKFs state
VectorXd SensorFusionLogic::getState() {
    return this->UKalmanFilter.getState();
}

/// Getter for the states covariance matrix
/// \return MatrixXd of the current UKFs state covariance
MatrixXd SensorFusionLogic::getCovariance() {
    return this->UKalmanFilter.getStateCovariance();
}


/// Perform the prediction step of the kalman filter
/// \param newTime time of the measurement
/// \return MatrixXd of the predicted sigma points
MatrixXd SensorFusionLogic::runKalmanFilterPrediction(ros::Time newTime) {
    //! Predict step which predicts the set of state sigma points
    MatrixXd predictedSigmaPoints = UKalmanFilter.predict((newTime - this->pointInTime).toSec());

    if (!UKalmanFilter.isMatrixValid(predictedSigmaPoints)) {
        throw ("Kalmanfilter Prediction Failed!");
    }

    this->pointInTime = newTime;
    return predictedSigmaPoints;
}

/// Performs the update step of the kalman filter
/// \param measuredState VectorXd containing the original measurements
/// \param measurementNoise MatrixXd containing the noise matrix of the Sensor
/// \param measurementSigmaPoints MatrixXd containing the sigma points from the measurements
/// \param predictedSigmaPoints MatrixXd containing the predicted sigma points
/// \param measurementDimension int of the measurement vector dimension
/// \param indexOfMeasuredYaw int of the index of the measured vectors yaw
///                          (use any value below 0 or above 5 to indicate that there is no yaw)
/// \return VectorXd of the updated state
VectorXd SensorFusionLogic::runKalmanFilterUpdate(VectorXd &measuredState, MatrixXd &measurementNoise,
                                                  MatrixXd &measurementSigmaPoints, MatrixXd &predictedSigmaPoints,
                                                  const int &measurementDimension, const int &indexOfMeasuredYaw) {
    UKalmanFilter.update(measuredState, measurementNoise, measurementSigmaPoints, predictedSigmaPoints,
                         measurementDimension, indexOfMeasuredYaw);

    VectorXd calculatedState = UKalmanFilter.getState();
    if (!UKalmanFilter.isVectorValid(calculatedState)) {
        throw ("Kalmanfilter Update Failed!");
    }

    return calculatedState;
}
