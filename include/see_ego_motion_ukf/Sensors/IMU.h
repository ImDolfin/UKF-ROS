#ifndef SEE_EGO_MOTION_CPP_IMU_H
#define SEE_EGO_MOTION_CPP_IMU_H

#include"Sensor.h"
#include"Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
 * The data class IMU contains the IMU relevant data
 * it inherits from the abstract "Sensor" Class
 */
class IMU : public Sensor {
public:
    /// IMU constructor
    IMU(double standardDeviationAcceleration = 1, double standardDeviationYaw = 0.5 , double standardDeviationYawRate = 0.5);

    /// IMU destructor
    ~IMU() = default;

    /// converts the RAW measurements to an actually usable set of values
    /// \return the converted measurements as VectorXd
    VectorXd getMeasurements();

    /// setter for a new set of RAW measurements
    /// \param rawMeasurements VectorXd of the sensors measurements in the order (first to last):
    ///                        aX, aY, aZ, yawRate, quaternionW, qX, qY, qZ
    /// \param covarianceMatrix MatrixXd containing the measurements covariance from [yaw, yaw rate, acceleration]
    /// \param timestamp       time stamp of the new measurement
    void setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp);

    /// transforms a set of sigma points to the measurement space
    /// \param predictedSigmaPoints set of predicted sigma points which will be predicated to measurement space
    /// \return transformed measurement sigma points
    MatrixXd transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints);

};


#endif //SEE_EGO_MOTION_CPP_IMU_H
