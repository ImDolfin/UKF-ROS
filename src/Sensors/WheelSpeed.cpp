#include "see_ego_motion_ukf/Sensors/IMU.h"
#include "see_ego_motion_ukf/Sensors/WheelSpeed.h"
#include"Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

/// WheelSpeed constructor
WheelSpeed::WheelSpeed(double standardDeviationVelocity) {
    //! estimated standard deviations of the different measurements
    double std_velocity = standardDeviationVelocity;

    //! measurement matrix that converts the state vector dimension to the measured values dimension
    this->measurementMatrix.resize(1, 6);
    this->measurementMatrix << 0, 0, 0, 0, 1, 0; //velocity
    //! contains standard deviations
    this->measurementNoise.resize(1, 1);
    this->measurementNoise << pow(std_velocity, 2);
    //! dimension of the measured valued
    this->measurementDimension = 1;
    //! The sensors calculated covariance matrix
    this->measurementCovariance = measurementNoise;
}

/// converts the RAW measurements to an actually usable set of values
/// \return the converted measurements as VectorXd
VectorXd WheelSpeed::getMeasurements() {
    double frontLeft = this->rawMeasurement(0);
    double frontRight = this->rawMeasurement(1);
    double rearLeft = this->rawMeasurement(2);
    double rearRight = this->rawMeasurement(3);

    //! calculate Total velocity
    double velocityTotal = sqrt(pow(frontLeft, 2) + pow(frontRight, 2) + pow(rearLeft, 2) + pow(rearRight, 2));
    velocityTotal = (frontLeft + frontRight)/2;

    //! assign measurements to output vector
    VectorXd returnVector(6);
    returnVector << 0.0,
                    0.0,
                    0.0,
                    0.0,
                    velocityTotal,
                    0.0;
    return returnVector;
}

/// setter for a new set of RAW measurements
/// \param rawMeasurements VectorXd of the sensors measurements in the order (first to last):
///                        front_left, front_right, rear_left, rear_right
/// \param covarianceMatrix MatrixXd containing the measurements covariance from [velocity]
/// \param timestamp       time stamp of the new measurement
void
WheelSpeed::setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp) {
    if (!Sensor::isVectorCorrect(rawMeasurements)) {
        throw std::invalid_argument("Invalid Vector Input!");
    } else if (!Sensor::isMatrixCorrect(covarianceMatrix)) {
        throw std::invalid_argument("Invalid Matrix Input!");
    }
    this->rawMeasurement = rawMeasurements;
    this->measurementCovariance = covarianceMatrix;
    this->timeStamp = timestamp;

}

/// transforms a set of sigma points to the measurement space
/// \return transformed measurement sigma points
MatrixXd WheelSpeed::transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints) {
    if (!Sensor::isMatrixCorrect(predictedSigmaPoints)) {
        throw std::invalid_argument("Invalid Input!");
    }
    MatrixXd transformedPoints(this->measurementDimension, predictedSigmaPoints.cols());
    //! extract the rows which are of relevance for the measured values
    transformedPoints.row(0) = predictedSigmaPoints.row(4); // velocity

    return transformedPoints;
}
