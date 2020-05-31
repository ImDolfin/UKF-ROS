
#ifndef SEE_EGO_MOTION_CPP_WHEELSPEED_H
#define SEE_EGO_MOTION_CPP_WHEELSPEED_H

#include"Sensor.h"

/*!
 * The data class WheelSpeed contains the WheelSpeed relevant data
 * it inherits from the abstract "Sensor" Class
 */
class WheelSpeed : public Sensor {
public:
    /// constructor
    WheelSpeed(double standardDeviationVelocity = 0.5);

    /// destructor
    ~WheelSpeed() = default;

    /// converts the RAW measurements to an actually usable set of values
    /// \return the converted measurements as VectorXd
    VectorXd getMeasurements();

    /// setter for a new set of RAW measurements
    /// \param rawMeasurements VectorXd of the sensors measurements in the order (first to last):
    ///                        front_left, front_right, rear_left, rear_right
    /// \param covarianceMatrix MatrixXd containing the measurements covariance from [velocity]
    /// \param timestamp       time stamp of the new measurement
    void setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp);

    /// transforms a set of sigma points to the measurement space
    /// \param predictedSigmaPoints set of predicted sigma points which will be predicated to measurement space
    /// \return transformed measurement sigma points
    MatrixXd transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints);

private:

};


#endif //SEE_EGO_MOTION_CPP_WHEELSPEED_H
