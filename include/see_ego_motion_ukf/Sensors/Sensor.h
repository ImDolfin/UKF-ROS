#ifndef SEE_EGO_MOTION_CPP_SENSOR_H
#define SEE_EGO_MOTION_CPP_SENSOR_H

#include"Eigen/Dense"
#include "ros/ros.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
 * Abstract class for data classes that represent physical sensors
 */
class Sensor {
public:
    /// Sensor constructor
    Sensor();

    /// Sensor destructor
    ~Sensor() = default;

    /// converts the RAW measurements to an actually usable set of values
    /// \return the converted measurements as VectorXd
    virtual VectorXd getMeasurements() = 0;

    /// getter for the timeStamp of the latest measurement
    /// \return timeStamp of type ros::Time
    ros::Time getTimeStamp();

    /// setter for a new set of RAW measurements
    /// \param rawMeasurements VectorXd containing the raw measurements input
    /// \param covarianceMatrix MatrixXd containing the measurements covariance
    /// \param timestamp timestamp of the raw measurement input
    virtual void
    setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp) = 0;

    /// getter for the measurement noise matrix
    /// \return measurement noise as MatrixXd
    MatrixXd getMeasurementNoise();

    /// getter for the measurement covariance matrix of the sensor
    /// \return measurement covariance as MatrixXd
    MatrixXd getMeasurementCovariance();

    /// getter for the measurement matrix
    /// \return measurement matrix as MatrixXd
    MatrixXd getMeasurementMatrix();

    /// Getter for the dimension of the measurements
    /// \return int of the measurement dimension
    int getMeasurementDimension();

    /// transforms a set of sigma points to the measurement space
    /// \return transformed measurement sigma points
    virtual MatrixXd transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints) = 0;

    /// Tests a Matrix for NAN and Inf values
    /// \param testMatrix MatrixXd which is supposed to be tested
    /// \return True if Matrix passes the test, False if it doesn't
    bool isMatrixCorrect(const MatrixXd testMatrix);

    /// Tests a Vector for NAN and Inf values
    /// \param testVector VectorXd which is supposed to be tested
    /// \return True if Vector passes the test, False if it doesn't
    bool isVectorCorrect(const VectorXd testVector);

protected:
    /// timestamp of the latest measurement
    ros::Time timeStamp;
    /// dimension of the measurements
    int measurementDimension;
    /// standard deviation noise values of the individual sensors.
    /// The Noise values should not be altered because they are values given by the manufacturer
    MatrixXd measurementNoise;
    /// The sensors calculated covariance matrix
    MatrixXd measurementCovariance;
    /// measurement matrix that converts the state vector dimension to the measured values dimension
    MatrixXd measurementMatrix;
    /// RAW measurement vector
    VectorXd rawMeasurement;
};

#endif //SEE_EGO_MOTION_CPP_SENSOR_H
