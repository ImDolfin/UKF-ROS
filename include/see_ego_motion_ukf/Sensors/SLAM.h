#ifndef SEE_EGO_MOTION_CPP_SLAM_H
#define SEE_EGO_MOTION_CPP_SLAM_H

#include"Eigen/Dense"
#include "ros/ros.h"
#include"Sensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
 * Class that contains the input SlAM state data
 */
class SLAM: public Sensor {
public:
    /// SLAM constructor
    SLAM(double standardDeviationXPosition = 1, double standardDeviationYPosition = 0.5);

    /// SLAM destructor
    ~SLAM() = default;

    /// converts the RAW measurements to an actually usable set of values
    /// \return the converted measurements as VectorXd
    VectorXd getMeasurements();

    /// setter for a new set of RAW measurements
    /// \param rawMeasurements VectorXd of the sensors measurements
    /// \param covarianceMatrix MatrixXd containing the measurements covariance from
    /// \param timestamp       time stamp of the new measurement
    void setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp);

    /// transforms a set of sigma points to the measurement space
    /// \param predictedSigmaPoints set of predicted sigma points which will be predicated to measurement space
    /// \return transformed measurement sigma points
    MatrixXd transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints);

};


#endif //SEE_EGO_MOTION_CPP_SLAM_H
