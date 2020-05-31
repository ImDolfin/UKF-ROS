#ifndef SEE_EGO_MOTION_CPP_GPS_H
#define SEE_EGO_MOTION_CPP_GPS_H

#include"Sensor.h"
#include"Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
 * The data class GPS contains the GPS relevant data
 * it inherits from the abstract "Sensor" Class
 */
class GPS : public Sensor {
public:
    /// GPS constructor
    GPS(double standardDeviationXposition = 1, double standardDeviationYposition = 1);

    /// GPS destructor
    ~GPS() = default;

    /// converts the RAW measurements to an actually usable set of values
    /// \return the converted measurements as VectorXd
    VectorXd getMeasurements();

    /// setter for a new set of RAW measurements
    /// \param rawMeasurements VectorXd of the raw measurements in [longitude, latitude, altitude]
    /// \param covarianceMatrix MatrixXd containing the measurements covariance in [ENU x, ENU y]
    /// \param timestamp Timestamp of the new measurement
    void setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp);

    /// transforms a set of sigma points to the measurement space
    /// \param predictedSigmaPoints set of predicted sigma points which will be predicated to measurement space
    /// \return transformed measurement sigma points
    MatrixXd transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints);

    void setReferencePoints(const double &latitude, const double &longitude, const double &altitude);

private:
    /// radius of earth around the equator
    double equatorialRadius;
    /// radius of earth around the poles
    double polarRadius;

    double latitudeReferencePoint = 0;
    double longitudeReferencePoint = 0;
    double altitudeReferencePoint = 0;

    bool areReferencePointsSet = false;

    /// converts the geodetic values to the East-North-Up coordinate system
    /// \param latitude double geodetic latitude in degree
    /// \param longitude double geodetic longitude in degree
    /// \param altitude double geodetic altitude in m
    /// \return VectorXd containing the converted ENU values as [X, Y, Z]
    VectorXd convertGeodeticToENU(const double &latitude, const double &longitude, const double &altitude);

    /// converts the geodetic values to the earth-centered coordinate system
    /// \param latitude double geodetic latitude in degree
    /// \param longitude double geodetic longitude in degree
    /// \param altitude double geodetic altitude in m
    /// \return VectorXd containing the converted ECEF values as [X, Y, Z]
    VectorXd convertGeodeticToECEF(const double &latitude, const double &longitude, const double &altitude);

    /// convert earth-centered coordinate system to the East-North-Up coordinate system
    /// \param ecefValues VectorXd containing the [X, Y, Z] positions in the ECEF coordinate system
    /// \param latitude double geodetic latitude
    /// \param longitude double geodetic longitude
    /// \param altitude double geodetic height
    /// \return VectorXd containing the converted ENU values as [East, North, Up]
    VectorXd convertECEFoENU(const VectorXd &ecefValues);

    /// converts a degree value to a radiant
    /// \param degreeValue double value that is supposed to be converted
    /// \return double of the converted radiant value
    double convertDegreesToRadians(const double &degreeValue);
};


#endif //SEE_EGO_MOTION_CPP_GPS_H
