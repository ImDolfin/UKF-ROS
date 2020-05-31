#include "see_ego_motion_ukf/Sensors/SensorNullObject.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

/// SensorNullObject constructor
SensorNullObject::SensorNullObject(){
    //! measurement matrix that converts the state vector dimension to the measured values dimension
    this->measurementMatrix.resize(6,6);
    this->measurementMatrix.setZero();

    //! contains standard deviations
    this->measurementNoise.resize(2,2);
    this->measurementNoise.setZero();
    //! dimension of the measured valued
    this->measurementDimension = 6;
    //! The sensors calculated covariance matrix
    this->measurementCovariance = measurementNoise;
}

/// converts the RAW measurements to an actually usable set of values
/// \return the converted measurements as VectorXd
VectorXd SensorNullObject::getMeasurements(){
    //! assign measurements to output vector
    VectorXd returnVector(6);
    returnVector.setZero();
    return returnVector;
}
/// setter for a new set of RAW measurements
/// \param rawMeasurements VectorXd of the sensors measurements
/// \param covarianceMatrix MatrixXd containing the measurements covariance
/// \param timestamp       time stamp of the new measurement
void SensorNullObject::setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp){
    if(!Sensor::isVectorCorrect(rawMeasurements)){
        throw std::invalid_argument("Invalid Vector Input!");
    }
    else if(!Sensor::isMatrixCorrect(covarianceMatrix)){
        throw std::invalid_argument("Invalid Matrix Input!");
    }
    this->rawMeasurement = rawMeasurements;
    this->measurementCovariance = covarianceMatrix;
    this->timeStamp = timestamp;

}

/// transforms a set of sigma points to the measurement space
/// \return transformed measurement sigma points
MatrixXd SensorNullObject::transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints){
    if(!Sensor::isMatrixCorrect(predictedSigmaPoints)){
        throw std::invalid_argument("Invalid Input!");
    }
    MatrixXd transformedPoints = predictedSigmaPoints;
    return transformedPoints;
}