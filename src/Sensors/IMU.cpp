#include "see_ego_motion_ukf/Sensors/IMU.h"
#include"Eigen/Dense"
#include "Eigen/Geometry"
//#include <gfr_common/transform.h>
#include <tf2/utils.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

/// IMU constructor
IMU::IMU(double standardDeviationAcceleration, double standardDeviationYaw , double standardDeviationYawRate){
    //! estimated standard deviations of the different measurements
    double std_acceleration = standardDeviationAcceleration;
    double std_yaw = standardDeviationYaw;
    double std_yawRate = standardDeviationYawRate;

    //! measurement matrix that converts the state vector dimension to the measured values dimension
    this->measurementMatrix.resize(3,6);
    this->measurementMatrix <<  0, 0, 1, 0, 0, 0, //yaw
                                0, 0, 0, 1, 0, 0, //yawRate
                                0, 0, 0, 0, 0, 1; //acceleration
    //! contains standard deviations
    this->measurementNoise.resize(3,3);
    this->measurementNoise <<   pow(std_yaw,2)      ,           0             ,       0                ,
                                        0           , pow(std_yawRate,2)      ,       0                ,
                                        0           ,           0             , pow(std_acceleration,2);
    //! dimension of the measured valued
    this->measurementDimension = 3;
    //! The sensors calculated covariance matrix
    this->measurementCovariance = measurementNoise;
}

/// converts the RAW measurements to an actually usable set of values
/// \return the converted measurements as VectorXd
VectorXd IMU::getMeasurements(){
    double accelerationX = this->rawMeasurement(0);
    double accelerationY = this->rawMeasurement(1);
    //double accelerationZ = this->rawMeasurement(2);//-9.81;
    double yawRate = this->rawMeasurement(3);

    //! generate quaternion out of the input
    geometry_msgs::Quaternion quaternion;
    quaternion.x = this->rawMeasurement(5); //x
    quaternion.y = this->rawMeasurement(6); //y
    quaternion.z = this->rawMeasurement(7); //z
    quaternion.w = this->rawMeasurement(4); //w
    //convert quaternion to yaw
    double yaw = tf2::getYaw(quaternion);
    //! calculate Total acceleration
    //https://physics.stackexchange.com/questions/41653/how-do-i-get-the-total-acceleration-from-3-axes
    double accelerationTotal = sqrt(pow(accelerationX,2)+pow(accelerationY,2)/*+pow(accelerationZ,2)*/);

    //! assign measurements to output vector
    VectorXd returnVector(6);
    returnVector    <<  0.0,
                        0.0,
                        yaw,
                        yawRate,
                        0.0,
                        accelerationTotal;
    return returnVector;
}
/// setter for a new set of RAW measurements
/// \param rawMeasurements VectorXd of the sensors measurements in the order (first to last):
///                        aX, aY, aZ, yawRate, quaternionW, qX, qY, qZ
/// \param covarianceMatrix MatrixXd containing the measurements covariance from [yaw, yaw rate, acceleration]
/// \param timestamp       time stamp of the new measurement
void IMU::setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp){
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
MatrixXd IMU::transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints){
    if(!Sensor::isMatrixCorrect(predictedSigmaPoints)){
        throw std::invalid_argument("Invalid Input!");
    }
    MatrixXd transformedPoints(this->measurementDimension, predictedSigmaPoints.cols());
    //! extract the rows which are of relevance for the measured values
    transformedPoints.row(0) = predictedSigmaPoints.row(2); // yaw
    transformedPoints.row(1) = predictedSigmaPoints.row(3); // yawRate
    transformedPoints.row(2) = predictedSigmaPoints.row(5); // acceleration

    return transformedPoints;
}
