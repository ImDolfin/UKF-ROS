#include "see_ego_motion_ukf/Sensors/SLAM.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

/// SLAM constructor
SLAM::SLAM(double standardDeviationXPosition, double standardDeviationYPosition){
    //! estimated standard deviations of the different measurements
    double std_xPosition = standardDeviationXPosition;
    double std_yPosition = standardDeviationYPosition;
    //! measurement matrix that converts the state vector dimension to the measured values dimension
    this->measurementMatrix.resize(2,6);
    this->measurementMatrix <<  1, 0, 0, 0, 0, 0, //PositionX
                                0, 1, 0, 0, 0, 0; //PositionY

    //! contains standard deviations
    this->measurementNoise.resize(2,2);
    this->measurementNoise <<   pow(std_xPosition,2)        ,           0               ,
                                                0           , pow(std_yPosition,2)      ;
    //! dimension of the measured valued
    this->measurementDimension = 2;
    //! The sensors calculated covariance matrix
    this->measurementCovariance = measurementNoise;
}

/// converts the RAW measurements to an actually usable set of values
/// \return the converted measurements as VectorXd
VectorXd SLAM::getMeasurements(){
    //! assign measurements to output vector
    VectorXd returnVector(6);
    returnVector    <<  0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0;
    return returnVector;
}
/// setter for a new set of RAW measurements
/// \param rawMeasurements VectorXd of the sensors measurements
/// \param covarianceMatrix MatrixXd containing the measurements covariance
/// \param timestamp       time stamp of the new measurement
void SLAM::setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp){
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
MatrixXd SLAM::transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints){
    if(!Sensor::isMatrixCorrect(predictedSigmaPoints)){
        throw std::invalid_argument("Invalid Input!");
    }
    MatrixXd transformedPoints(this->measurementDimension, predictedSigmaPoints.cols());
    //! extract the rows which are of relevance for the measured values
    transformedPoints.row(0) = predictedSigmaPoints.row(0); // xPosition
    transformedPoints.row(1) = predictedSigmaPoints.row(1); // yPosition


    return transformedPoints;
}