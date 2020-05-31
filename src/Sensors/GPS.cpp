#include "see_ego_motion_ukf/Sensors/GPS.h"
#include <Eigen/Dense>
#include <stdexcept>

using Eigen::MatrixXd;
using Eigen::VectorXd;


GPS::GPS(double standardDeviationXposition, double standardDeviationYposition){
    double std_XPosition = standardDeviationXposition; // estimate of 2 standard_deviation
    double std_YPosition = standardDeviationYposition; // estimate of 2 standard_deviation

    equatorialRadius = 6378137.0; // a - as usually named for formulas
    polarRadius = 6356752.3142; // b - as usually named for formulas

    /// standard deviation noise values
    /// standard deviation noise values of the individual sensors.
    /// The Noise values should not be altered because they are values given by the manufacturer
    this->measurementNoise.resize(2,2);
    this->measurementNoise <<   pow(std_XPosition, 2),         0          ,
                                        0            , pow(std_YPosition, 2);

    /// measurement matrix that converts the state vector dimension to the measured values dimension
    this->measurementMatrix.resize(2,6);
    this->measurementMatrix <<  1 , 0 , 0 , 0 , 0 , 0,
                                0 , 1 , 0 , 0 , 0 , 0;

    ///  RAW measurement vector
    this->rawMeasurement.resize(3);
    this->rawMeasurement.fill(0.0);

    /// dimension of the measurements
    this->measurementDimension = 2;

    //! The sensors calculated covariance matrix
    this->measurementCovariance = this->measurementNoise;


}

void GPS::setReferencePoints(const double &latitude, const double &longitude, const double &altitude){
    this->latitudeReferencePoint = latitude;
    this->longitudeReferencePoint = longitude;
    this->altitudeReferencePoint = altitude;
    this->areReferencePointsSet = true;
}

/// converts the RAW measurements to an actually usable set of values
/// \return the converted measurements as VectorXd
VectorXd GPS::getMeasurements(){
    if(!areReferencePointsSet){
      setReferencePoints(rawMeasurement(0), rawMeasurement(1), rawMeasurement(2));
    }
    VectorXd transformedMeasurements(6);
    transformedMeasurements.fill(0.0);
    VectorXd enuCoordinates = convertGeodeticToENU(rawMeasurement(0), rawMeasurement(1), rawMeasurement(2));

    if(!Sensor::isVectorCorrect(enuCoordinates)){
      throw std::invalid_argument("Geodetic to ENU conversion failed!");
    }
    transformedMeasurements(0) = enuCoordinates(1);
    transformedMeasurements(1) = enuCoordinates(0);

    return transformedMeasurements;
}

/// converts the geodetic values to the East-North-Up coordinate system
/// \param latitude double geodetic latitude in degree
/// \param longitude double geodetic longitude in degree
/// \param altitude double geodetic altitude in m
/// \return VectorXd containing the converted ENU values as [X, Y, Z]
VectorXd GPS::convertGeodeticToENU(const double &latitude, const double &longitude, const double &altitude){
    VectorXd ecefCoordinates = convertGeodeticToECEF(latitude, longitude, altitude);
    //return ecefCoordinates;
    return convertECEFoENU(ecefCoordinates);
}

/// converts the geodetic values to the earth-centered coordinate system
/// \param latitude double geodetic latitude in degree
/// \param longitude double geodetic longitude in degree
/// \param altitude double geodetic altitude in m
/// \return VectorXd containing the converted ECEF values as [X, Y, Z]
VectorXd GPS::convertGeodeticToECEF(const double &latitude, const double &longitude, const double &altitude){
    /// convert degrees to radians
    double lat = convertDegreesToRadians(latitude);
    double lon = convertDegreesToRadians(longitude);

    double firstNumericalEccentricitySquared = 1 - pow(polarRadius, 2) / pow(equatorialRadius, 2); // e squared
    double primeVerticalRadiusOfCurvature = equatorialRadius / sqrt( 1 - firstNumericalEccentricitySquared * sin(lat) * sin(lat)); //N(lat)
    //double firstNumericalEccentricitySquared = pow(polarRadius, 2) / pow(equatorialRadius, 2); // e squared
    //double primeVerticalRadiusOfCurvature = equatorialRadius / sqrt(pow(equatorialRadius,2)* pow(cos(latitude), 2) + pow(polarRadius, 2) * pow(sin(latitude), 2)); //sqrt( firstNumericalEccentricitySquared * sin(lat) * sin(lat)); //N(lat)

    VectorXd ECEF(3);
    //X
    ECEF(0) = (primeVerticalRadiusOfCurvature + altitude) * cos(lat) * cos(lon);
    //Y
    ECEF(1) = (primeVerticalRadiusOfCurvature + altitude) * cos(lat) * sin(lon);
    //Z
    ECEF(2) = (pow(polarRadius, 2)/pow(equatorialRadius, 2) * primeVerticalRadiusOfCurvature + altitude) * sin(lat);
    //ECEF(2) = ((1-firstNumericalEccentricitySquared) * primeVerticalRadiusOfCurvature + altitude) * sin(lat);

    return ECEF;
}

/// convert earth-centered coordinate system to the East-North-Up coordinate system
/// \param ecefValues VectorXd containing the [X, Y, Z] positions in the ECEF coordinate system
/// \param latitude double geodetic latitude
/// \param longitude double geodetic longitude
/// \param altitude double geodetic height
/// \return VectorXd containing the converted ENU values as [East, North, Up]
VectorXd GPS::convertECEFoENU(const VectorXd &ecefValues){
    /// assign ECEF vector values to seperate variables for better readability
    double ecefX = ecefValues(0);
    double ecefY = ecefValues(1);
    double ecefZ = ecefValues(2);

    /// convert degrees to radians
    double lambda = convertDegreesToRadians(this->latitudeReferencePoint);
    double phi = convertDegreesToRadians(this->longitudeReferencePoint);

    MatrixXd conversionMatrix(3,3);
    conversionMatrix << -sin(lambda)           ,       cos(lambda)      ,     0   ,
                        -cos(lambda) * sin(phi) , -sin(lambda) * sin(phi), cos(phi),
                        cos(lambda) * cos(phi),  sin(lambda) * cos(phi), sin(phi);

    VectorXd referenceECEF = convertGeodeticToECEF(this->latitudeReferencePoint, this->longitudeReferencePoint, this->altitudeReferencePoint);
    VectorXd location(3);
    location.fill(0.0);
    location(0) = ecefX - referenceECEF(0);
    location(1) = ecefY - referenceECEF(1);
    location(2) = ecefZ - referenceECEF(2);

    VectorXd ENU = conversionMatrix * location;
    return ENU;
}

/// converts a value from degree to radiant
/// \param degreeValue double value that is supposed to be converted
/// \return double of the converted radiant value
double GPS::convertDegreesToRadians(const double &degreeValue){
    return (degreeValue * M_PI) / 180;
}

/// setter for a new set of RAW measurements
/// \param rawMeasurements VectorXd of the raw measurements in [longitude, latitude, altitude]
/// \param covarianceMatrix MatrixXd containing the measurements covariance in [ENU x, ENU y]
/// \param timestamp Timestamp of the new measurement
void GPS::setRawInput(const VectorXd rawMeasurements, const MatrixXd covarianceMatrix, const ros::Time timestamp){
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
/// \param predictedSigmaPoints set of predicted sigma points which will be predicated to measurement space
/// \return transformed measurement sigma points
MatrixXd GPS::transformPointsToMeasurementSpace(const MatrixXd &predictedSigmaPoints){
    if(!Sensor::isMatrixCorrect(predictedSigmaPoints)){
        throw std::invalid_argument("Invalid Input!");
    }
    MatrixXd measurementSigmaPoints(this->measurementDimension, predictedSigmaPoints.cols());
    measurementSigmaPoints.row(0) = predictedSigmaPoints.row(0);
    measurementSigmaPoints.row(1) = predictedSigmaPoints.row(1);
    return measurementSigmaPoints;
}
