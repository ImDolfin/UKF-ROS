#include "see_ego_motion_ukf/Sensors/Sensor.h"
#include <Eigen/Dense>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Sensor constructor
Sensor::Sensor() {
    timeStamp = ros::Time::now();
}

/// getter for the timeStamp of the latest measurement
/// \return timeStamp of type ros::Time
ros::Time Sensor::getTimeStamp() {
    return timeStamp;

}

/// getter for the measurement error matrix
/// \return measurement error as MatrixXd
MatrixXd Sensor::getMeasurementMatrix() {
    return measurementMatrix;
}

/// Getter for the dimension of the measurements
/// \return int of the measurement dimension
int Sensor::getMeasurementDimension() {
    return measurementDimension;
}

/// getter for the measurement noise matrix
/// \return measurement noise as MatrixXd
MatrixXd Sensor::getMeasurementNoise() {
    return measurementNoise;
}

/// getter for the measurement covariance matrix of the sensor
/// \return measurement covariance as MatrixXd
MatrixXd Sensor::getMeasurementCovariance() {
    if (measurementCovariance.determinant() == 0 || !isMatrixCorrect(measurementCovariance)) {
        return measurementNoise;
    }
    return measurementCovariance;
}

/// Tests a Matrix for NAN and Inf values
/// \param testMatrix MatrixXd which is supposed to be tested"
/// \return True if Matrix passes the test, False if it doesn't
bool Sensor::isMatrixCorrect(const MatrixXd testMatrix) {
    for (int i = 0; i < testMatrix.rows(); i++) {
        for (int j = 0; j < testMatrix.cols(); j++) {
            if (std::isnan(testMatrix(i, j)) || std::isinf(testMatrix(i, j))) {
                return false;
            }
        }
    }
    return true;
}

/// Tests a Vector for NAN and Inf values
/// \param testVector VectorXd which is supposed to be tested
/// \return True if Vector passes the test, False if it doesn't
bool Sensor::isVectorCorrect(const VectorXd testVector) {
    for (int i = 0; i < testVector.size(); i++) {
        if (std::isnan(testVector(i)) || std::isinf(testVector(i))) {
            return false;
        }
    }
    return true;
}
