#include "see_ego_motion_ukf/UnscentedKalmanFilter.h"
#include <Eigen/Dense>
#include <cmath>
#include <ros/ros.h>
#include <iostream>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
* Constructor
*/
UnscentedKalmanFilter::UnscentedKalmanFilter(){
    //! scale parameter in the range of (0,1]
    parameters.alpha = 0.8;

    //! used to incorporate prior knowledge of x's distribution -> 2 is optimal for gaussians
    parameters.beta  = 2;

    //! dimension of the state vector
    parameters.stateDimension = 6;

    //! scale parameter -> usually 0 or 3-stateDimension
    parameters.kappa = 3 - parameters.stateDimension;

    //! amount of sigma points that have to be generated
    parameters.sigmaPointAmount = parameters.stateDimension*2+1;

    //! scale parameter resulting out of alpha sqrt(alpha)*(stateDimension+kappa)-statedimension
    parameters.lambda = pow(parameters.alpha, 2) * (parameters.stateDimension + parameters.kappa) - parameters.stateDimension;

    //! process noise standard deviation yaw acceleration
    parameters.std_YawRate = 1;

    //! process noise standard deviation longitudinal acceleration
    parameters.std_Acceleration = 6;

    //! boolean stating the initialization status -> false per default until initialized
    initialized = false;

    //! state vector with values: [xPosition, yPosition, yaw(Psi), yaw rate(dPsi), velocity(v), acceleration(a)]
    state.resize(parameters.stateDimension);
    state.fill(0.0);
    //! covariance matrix of the state
    stateCovariance.resize(parameters.stateDimension, parameters.stateDimension);
    stateCovariance.fill(0.0);
    //! matrix containing the predicted sigma points
    predictedSigmaPoints.resize(parameters.stateDimension, parameters.sigmaPointAmount);
    predictedSigmaPoints.fill(0.0);
}

/*!
* Destructor
*/
UnscentedKalmanFilter::~UnscentedKalmanFilter() {}

/// Getter for the initialization status of the unscented kalman filter
/// \return true if initialized; False if not
bool UnscentedKalmanFilter::isInitialized(){
    return initialized;
}

/// Initializes the unscented kalman filter
/// \return true if initialized; False if not
bool UnscentedKalmanFilter::initialize(VectorXd firstState){
    //! check if the ukf has not already been initialized
    if (initialized == true)
    {
        return initialized;
    }
    if(!isVectorValid(firstState))
    {
        return false;
    }
    //! set state specific weights
    this->stateWeights = setStateWeights();
    //! set state covariance specific weights
    this->stateCovarianceWeights = setCovarianceWeights();
    //! assign the first measurement to the state and therefore initialize it with the first values
    this->state = firstState;

    this->stateCovariance = 100 * Eigen::MatrixXd::Identity(this->parameters.stateDimension, this->parameters.stateDimension);

    this->initialized = true;
    return initialized;
}

/// Getter for the state vector
/// \return VectorXd of the states vector
VectorXd UnscentedKalmanFilter::getState(){
    return this->state;
}

/// Getter for the state covariance matrix
/// \return MatrixXd of the state covariance
MatrixXd UnscentedKalmanFilter::getStateCovariance(){
    return this->stateCovariance;
}

/// Getter for the matrix that contains the predicted sigma points
/// \return MatrixXd of the sigma points
MatrixXd UnscentedKalmanFilter::getPredictedSigmaPoints(){
    return this->predictedSigmaPoints;
}

/// Predict step of the Kalman filter
/// Basically Unscented Transforms the old state to a new predicted mean and covariance
/// \param deltaTime elapsed time between states
/// \return predicted sigma points
MatrixXd UnscentedKalmanFilter::predict(const double deltaTime){
    //ROS_INFO("------------------------------------------start predict--------------------------------------------------");
    ROS_INFO_STREAM("deltaTime: \n" << deltaTime<< "\n");
    if (!isInitialized())
    {
        throw ("Unscented Kalman Filter has to be initialized first!");
    }
    std::string strin("");
    //ROS_INFO_STREAM("time delta: " + std::to_string(deltaTime));
    if(deltaTime < 0)
    {
        throw std::invalid_argument ("The provided Time interval is smaller than 0.0");
    }
    //ROS_INFO("generate Sigma Points");
    //! generate the set of sigma points out of the old state and state covariance
    MatrixXd sigmaPoints = generateSigmaPoints(this->state, this->stateCovariance);
    ROS_INFO_STREAM("Generated Sigma Points: \n" << sigmaPoints << "\n");
    //ROS_INFO("predictSigmaPoints");
    //! use these sigma points and transform them through the CTRA motion model
    this->predictedSigmaPoints = sigmaPointPrediction(sigmaPoints, deltaTime);
    ROS_INFO_STREAM("Predicted Sigma Points: \n" << this->predictedSigmaPoints<< "\n");

    //ROS_INFO("calculate State");
    //! first predict the new state out of the set of transformed(predicted) sigma points
    //ROS_INFO_STREAM("State: ");
    //ROS_INFO_STREAM(this->state.transpose());

    VectorXd predictedState = calculateState(this->state, this->predictedSigmaPoints);
    ROS_INFO_STREAM("Predicted State: \n" << predictedState.transpose() << "\n");

    //ROS_INFO("calculate process noise");
    //! second, calculate the process noise matrix from deltaTime, standard deviations and the predicted states yaw
    MatrixXd processNoise = calculateProcessNoise(deltaTime, predictedState(2));
    ROS_INFO_STREAM("processNoise: \n" << processNoise<< "\n");

    //ROS_INFO("predict covariance");
    //! third, predict the new state covariance matrix using
    //! the transformed(predicted) sigma points and the new predicted mean
    MatrixXd predictedStateCovariance = predictCovariance(this->stateCovariance, this->predictedSigmaPoints, predictedState, processNoise, 2);
    ROS_INFO_STREAM("Predicted state covariance: \n" << predictedStateCovariance << "\n");

    //! assign new state and state covariance
    if(!(isVectorValid(predictedState) || isMatrixValid(predictedStateCovariance))){
        throw std::invalid_argument("The prediction of the state failed because the state/state covariance could not be calculated properly and contains illegal values!");
    }
    this->state = predictedState;
    this->stateCovariance = predictedStateCovariance;
    //ROS_INFO("finished predict");
    return predictedSigmaPoints;
}


/// Generates the set of sigma points out of the given state vector and covariance matrix
/// \param initialState     the current state which the point will be centered around
/// \param initialStateCovariance state covariance of the state for calculation of the points
/// \return MatrixXd of sigma points
MatrixXd UnscentedKalmanFilter::generateSigmaPoints(VectorXd initialState, MatrixXd initialStateCovariance) {
    //! create sigma point matrix
    MatrixXd sigmaPoints(parameters.stateDimension, parameters.sigmaPointAmount);

    //! create square root matrix
    //! created by taking the factor L from the cholesky decomposition of the stateCovariance Matrix
    MatrixXd squareRootMatrix = initialStateCovariance.llt().matrixL();

    //! create sigma points and add them to the sigmaPoint Matrix
    //! sigmaPoints Matrix is size of stateDimension in height and (2*stateDimension+1) in length
    //! first sigmaPoint is always the state/mean
    sigmaPoints.col(0)  = initialState;
    //! (2*stateDimension) sigma points are calculated and assigned afterwards
    for (int i = 0; i < parameters.stateDimension; i++)
    {
        //! all sigma points for 0 to stateDimension are calculated like:
        //!                                                      x        + sqrt(    lambda           +      stateDimension         ) * column i of squareRootMatrix
        sigmaPoints.col(i+1)                           = initialState + sqrt((parameters.lambda + parameters.stateDimension)) * squareRootMatrix.col(i);
        //! all sigma points for stateDimension+1 to 2*stateDimension are calculated like:
        //!                                                      x        - sqrt(    lambda           +      stateDimension         ) * column i of squareRootMatrix
        sigmaPoints.col(i+1+parameters.stateDimension) = initialState - sqrt((parameters.lambda + parameters.stateDimension)) * squareRootMatrix.col(i);
    }
    return sigmaPoints;
}

/// Predict all Sigma Points through the CTRA motion model without taking Noise into account
/// \param sigmaPoints MatrixXd of the generated set of sigma points
/// \param deltaTime time interval between the old and the "to predict" state
/// \return MatrixXd of the predicted SigmaPoints
MatrixXd UnscentedKalmanFilter::sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime) {
    //! create a temporary predicted sigma point matrix
    MatrixXd predictedSigmaPoints(parameters.stateDimension, parameters.sigmaPointAmount);
    predictedSigmaPoints.fill(0.0);
    //! predict sigma points
    for (int i = 0; i < parameters.sigmaPointAmount; i++) {
        //! extract values for better readability
        //! [xPosition, yPosition, yaw(Psi), yaw rate(dPsi), velocity(v), acceleration(a)]
        const double xPosition = sigmaPoints(0, i);
        const double yPosition = sigmaPoints(1, i);
        const double yaw = sigmaPoints(2, i);
        const double yawRate= sigmaPoints(3, i);
        const double velocity = sigmaPoints(4, i);
        const double acceleration = sigmaPoints(5, i);

        /// CTRA Motion Model
        /// predicted state values
        double predictedStateValueX, predictedStateValueY;
        double predictedVelocity = velocity + acceleration * deltaTime;
        double predictedYaw = yaw + yawRate * deltaTime;

        //! calculate the state values using yaw and yaw rates instead of Vx,Vy, Ax, Ay
        //! avoid division by zero
        //! a yaw rate below 0.001 rad/sec equals an almost straight moving car e.g. yaw rate => is smaller than 1 degree/sec
        if (fabs(yawRate) < 0.001) {
            //! predict x and y using the yaw without the yawRate == 0, which equals an almost straight moving Car
            //! therefore the formula of kinematic is applicable with the use of yaw angles
            predictedStateValueX = xPosition + velocity * deltaTime * cos(yaw) + (1/2) * acceleration * pow(deltaTime, 2) * cos(yaw);
            predictedStateValueY = yPosition + velocity * deltaTime * sin(yaw) + (1/2) * acceleration * pow(deltaTime, 2) * sin(yaw);
        } else {
            //! the formula of kinematic is also applicable here with the use of yaw angles. However, here they changed over time, which results in the use
            //! of the yaw rate and difference between the old and the new yaw angle

            //CTRV
            //predictedStateValueX = xPosition + velocity/yawRate *(sin(yawRate*deltaTime+yaw)-sin(yaw));
            //predictedStateValueY = yPosition + velocity/yawRate *(-cos(yawRate*deltaTime+yaw)+cos(yaw));

            //CTRA
            predictedStateValueX = xPosition + (1 / (yawRate*yawRate)) * ((velocity * yawRate + acceleration * yawRate * deltaTime ) * sin(predictedYaw)
                    + acceleration * cos(predictedYaw) - velocity * yawRate * sin(yaw) - acceleration * cos(yaw));
            predictedStateValueY = yPosition + (1 / (yawRate*yawRate)) * ((-velocity * yawRate - acceleration * yawRate * deltaTime ) * cos(predictedYaw)
                    + acceleration * sin(predictedYaw) + velocity * yawRate * cos(yaw) - acceleration * sin(yaw));
        }

        //! the yaw rate and acceleration will not be changed because the CTRA motion model assumes that the turn rate and acceleration are constant.
        double predictedYawRate = yawRate;
        double predictedAcceleration = acceleration;

        //! But because both values will not be 100% constant for the most time they will require modelled noise.
        //! That noise will be added later by using the Q Matrix of the calculateProcessNoise() method

        //!write predicted sigma point into right column using the respective rows
        predictedSigmaPoints(0, i) = predictedStateValueX;
        predictedSigmaPoints(1, i) = predictedStateValueY;
        predictedSigmaPoints(2, i) = predictedYaw;
        predictedSigmaPoints(3, i) = predictedYawRate;
        predictedSigmaPoints(4, i) = predictedVelocity;
        predictedSigmaPoints(5, i) = predictedAcceleration;
    }
    return predictedSigmaPoints;
}

/// calculates the Process Noise of the new deltatime
/// \param deltaTime elapsed time between states
/// \param yaw predicted yaw angle of the predicted state vector
/// \return MatrixXd of the process noise matrix
MatrixXd UnscentedKalmanFilter::calculateProcessNoise(const double &deltaTime, const double &yaw){
    double varianceAcceleration = pow(parameters.std_Acceleration, 2);
    double varianceYawRate = pow(parameters.std_YawRate, 2);
    double deltaTimeSquared = pow(deltaTime,2);

    //! Matrix G containing the non random components of the Noise Component
    MatrixXd G(6,2);
    G <<    (deltaTimeSquared/2)*cos(yaw)  ,          0, //based on px + v * dT * cos(yaw) + 1/2 * a * dt^2 * cos(yaw)
            (deltaTimeSquared/2)*sin(yaw)  ,          0, //based on py + v * dT * sin(yaw) + 1/2 * a * dt^2 * sin(yaw)
                        0                  ,  deltaTime, //based on psi + dpsi * dT
                        0                  ,          1, //based on the assumption that turn rate (dpsi) = constant
                        deltaTime          ,          0, //based on v + a * dT
                        1                  ,          0; //based on the assumption that acceleration = constant
    //! Matrix Qv containing the random components of the Noise Component (std_YawRate and std_Acceleration)
    MatrixXd Qv(2,2);
    Qv <<   varianceAcceleration,               0, // based on standard deviation of acceleration
            0                   , varianceYawRate; // based on standard deviation of yawRate

    MatrixXd processNoise = G * Qv * G.transpose();

    return processNoise;
}

/// Updates the state with the transformed measurements
/// \param measurements VectorXd containing the original measurements
/// \param measurementNoise MatrixXd containing the noise matrix of the Sensor
/// \param measurementSigmaPoints MatrixXd containing the sigma points from the measurements
/// \param predictedSigmaPoints MatrixXd containing the predicted sigma points
/// \param measurementDimension int of the measurement vector dimension
/// \param indexOfMeasuredYaw int of the index of the measured vectors yaw
///                          (use any value below 0 or above 5 to indicate that there is no yaw)
void UnscentedKalmanFilter::update(VectorXd &measurements, MatrixXd &measurementNoise,
        MatrixXd &measurementSigmaPoints, MatrixXd predictedSigmaPoints, const int measurementDimension, const int indexOfMeasuredYaw) {
    //ROS_INFO("------------------------------------------start update--------------------------------------------------");
    if (!isInitialized()){
        throw std::invalid_argument("Unscented Kalman Filter has to be initialized first!");
    }
    if(!isVectorValid(measurements))
    {
      throw std::invalid_argument("The provided measurements vector contains NAN and Inf values!");
    }
    if(!doesVectorContainValues(measurements))
    {
        throw std::invalid_argument("The provided measurements vector doesn't contain measurements!");
    }
    if(!isMatrixValid(measurementNoise))
    {
      throw std::invalid_argument("The provided measurementNoise matrix contains NAN and Inf values!");
    }
    if(!isMatrixValid(measurementSigmaPoints))
    {
      throw std::invalid_argument("The provided measurementSigmaPoints matrix contains NAN and Inf values!");
    }
    if(!isMatrixValid(predictedSigmaPoints))
    {
      throw std::invalid_argument("The provided predictedSigmaPoints matrix contains NAN and Inf values!");
    }
    if(measurementDimension <= 0 || measurementDimension > parameters.stateDimension){
      throw std::invalid_argument("The provided measurementDimension value is not permitted!");
    }
    //ROS_INFO_STREAM("Measurement state: ");
    //ROS_INFO_STREAM(measurements.transpose());

    //! create required vector and matrizes for the measurements
    VectorXd predictedMeasurements(measurementDimension);
    predictedMeasurements.fill(0.0);
    MatrixXd measurementsCovariance(measurementDimension , measurementDimension);
    measurementsCovariance.fill(0.0);
    //! crossCorrelation that describes the correlation between the predicted state and the measured state
    MatrixXd crossCorrelation(parameters.stateDimension, measurementDimension);
    crossCorrelation.fill(0.0);

    //! predict/calculate the measurement state out of the measurement sigma points
    predictedMeasurements = calculateState(predictedMeasurements, measurementSigmaPoints);
    //predictedMeasurements = measurements;
    ROS_INFO_STREAM("Predicted Measurement State: \n" << predictedMeasurements.transpose() << "\n");

    //! predict the measurements covariance matrix using the measurements sigma points, measurements state and the noise matrix from the Sensor
    measurementsCovariance = predictCovariance(measurementsCovariance, measurementSigmaPoints, predictedMeasurements, measurementNoise, indexOfMeasuredYaw);

    ROS_INFO_STREAM("measurementsCovariance: \n" << measurementsCovariance << "\n");


    //! declare the "difference" vectors for measurements and the state
    VectorXd measurementDifference;
    measurementDifference.fill(0.0);
    VectorXd stateDifference;
    stateDifference.fill(0.0);
    //ROS_INFO("loop measurements sigma points");
    //! loop through the sigma points of the measurements and state to assign the correlation to the crossCorrelation Matrix
    for (int i = 0; i < parameters.sigmaPointAmount; i++) {
        // difference between the measurement sigma point and the predicted measurement
        measurementDifference = measurementSigmaPoints.col(i) - predictedMeasurements;

        // normalize the yaw angle of the measurement
        // it only makes sense to normalize the measurements yaw angle if the measurement contains one
        // otherwise, the normalization step can be ignored
        if(!(indexOfMeasuredYaw >= measurementDimension)) {
            measurementDifference(indexOfMeasuredYaw) = normalizeAngle(measurementDifference(indexOfMeasuredYaw), -M_PI, M_PI);
        }
        // difference between the state sigma point and the predicted state
        stateDifference = predictedSigmaPoints.col(i) - this->state;
        // normalize the yaw angle of the state difference vector
        stateDifference(2) = normalizeAngle(stateDifference(2), -M_PI, M_PI);

        //ROS_INFO_STREAM(stateDifference.transpose());

        // assigning correlation between the calculated differences using the covariance weights
        crossCorrelation += stateCovarianceWeights(i) * stateDifference * measurementDifference.transpose();
    }

    ROS_INFO_STREAM("Cross Correlation: \n" << crossCorrelation << "\n");

    if (measurementNoise.determinant() == 0){
        throw std::invalid_argument("Taking the inverse of a 0 Matrix is prohibited.");
    }

    //ROS_INFO("Calculating Kalman Gain");
    //! calculating the KalmanGain
    //! it describes the priority/importance, correctness and relation of the state and the measurements
    //! it contains values between 0 and 1
    //! K<0.5 --> put more trust and emphasis in the predicted state
    //! K>0.5 --> put more trust and emphasis in the measurements
    MatrixXd KalmanGain = crossCorrelation * measurementsCovariance.inverse();
    ROS_INFO_STREAM("KalmanGain: \n" << KalmanGain << "\n");

    //! calculate the measurement difference to use it for the state correction
    measurementDifference = measurements - predictedMeasurements;
    if(!(indexOfMeasuredYaw >= measurementDimension)) {
        measurementDifference(indexOfMeasuredYaw) = normalizeAngle(measurementDifference(indexOfMeasuredYaw), -M_PI, M_PI);
    }

    //ROS_INFO_STREAM("measurement difference: ");
    //ROS_INFO_STREAM(measurementDifference.transpose());
    //ROS_INFO("Assign State Stuff");
    //! correct the state based on the Kalman gain and measurements

    VectorXd tempState = this->state;

    tempState += KalmanGain * measurementDifference;
    tempState(2) = normalizeAngle(tempState(2), -M_PI, M_PI);
    ROS_INFO_STREAM("Measurement Covariance: \n" << measurementsCovariance << "\n");

    ROS_INFO_STREAM("Corrected State: \n" << tempState.transpose() << "\n");

    /*ROS_INFO_STREAM("Measurements: \n" << measurements.transpose());*/

    //! correct the states covariance based on the Kalman gain and the measurements Covariance
    MatrixXd tempStateCovariance = this->stateCovariance - (KalmanGain * measurementsCovariance * KalmanGain.transpose());
    //MatrixXd diagonalCovarianceAsState = tempStateCovariance.diagonal();
    //tempStateCovariance = diagonalCovarianceAsState.asDiagonal();

    ROS_INFO_STREAM("Corrected State Covariance: \n" << tempStateCovariance << "\n");

    if(!(isMatrixValid(tempStateCovariance) || isVectorValid(tempState))){
        throw std::invalid_argument("The update step failed because the state/state covariance could not be calculated properly and contains illegal values!");
    }
    this->state = tempState;
    this->stateCovariance = tempStateCovariance;
    //ROS_INFO("finished update");
}

/// Predict the covariance matrix
/// \param stateCovariance base covariance matrix for the prediction
/// \param predictedSigmaPoints MatrixXd which contains the set of predicted sigma points
/// \param predictedState VectorXd of the predicted state vector
/// \param processNoise MatrixXd of the procmeasurementDimensioness noise
/// \return MatrixXd of the predicted state covariance
MatrixXd UnscentedKalmanFilter::predictCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState, MatrixXd &processNoise, int indexOfMeasuredYaw) {

    MatrixXd predictedStateCovariance(stateCovariance.rows(), stateCovariance.cols());
    //! prefill the state covariance to set the double type
    predictedStateCovariance.fill(0.0);
    //! iterate over sigma points of the amount sigmaPointAmount
    for (int i = 0; i < parameters.sigmaPointAmount; i++) {

        //! calculate the difference between the predicted state and the ith predicted sigma point state
        VectorXd stateDifference = predictedSigmaPoints.col(i) - predictedState;
        if(!(indexOfMeasuredYaw >= parameters.stateDimension)) {
            stateDifference(indexOfMeasuredYaw) = normalizeAngle(stateDifference(2), -M_PI, M_PI);
        }
        //! add up the state differences to generate a covariance values
        predictedStateCovariance += this->stateCovarianceWeights(i) * stateDifference * stateDifference.transpose();
    }

    //! add process noise to the covariance matrix
    predictedStateCovariance = predictedStateCovariance + processNoise;

    return predictedStateCovariance;
}

/// Calculate the mean state vector using the sigma points and old state
/// \param state base state vector for the prediction
/// \param predictedSigmaPoints MatrixXd of the predicted sigma point set
/// \return VectorXd of predicted state vector
VectorXd UnscentedKalmanFilter::calculateState(VectorXd &state, MatrixXd &predictedSigmaPoints) {
    //! create temporary state vector
    VectorXd predictedState = state;
    //! fill the state vector with 0.0 double values to avoid integer cutting
    predictedState.fill(0.0);
    //! add up all rows of the sigma points and the old state to calculate the new predicted state
    for (int i = 0; i < parameters.sigmaPointAmount; i++) {  //iterate over sigma points
        predictedState += this->stateWeights(i) * predictedSigmaPoints.col(i);
    }
    return predictedState;
}

/// Sets the weights for the state of the Uncented Kalman Filter.
/// \return VectorXd of the states weights of size sigmaPointAmount
VectorXd UnscentedKalmanFilter::setStateWeights(){
    //! create temporary weight vector of size sigmaPointAmount (stateDimension*2+1)
    VectorXd weights(parameters.sigmaPointAmount);
    weights.fill(0.0);

    //! set weights for each sigmaPoint
    //! the first weight is calculated differently from the others because it is the weight for the mean
    double firstWeight = parameters.lambda / (parameters.lambda + parameters.stateDimension);
    weights(0) = firstWeight;
    //! 2*stateDimension weights because the "+1" is the first weight for the mean
    for (int i=1; i<parameters.sigmaPointAmount; i++) {
        double weight = 1 / (2 * (parameters.stateDimension + parameters.lambda));
        weights(i) = weight;
    }
    return weights;
}

/// Sets the weights for the states covariance of the Unscented Kalman Filter
/// \return VectorXd of the states covariance weights of size sigmaPointAmount
VectorXd UnscentedKalmanFilter::setCovarianceWeights(){
    //! create temporary weight vector of size sigmaPointAmount (stateDimension*2+1)
    VectorXd weights(parameters.sigmaPointAmount);
    weights.fill(0.0);
    //! set weights for each sigmaPoint
    //! the first weight is calculated differently from the others because it is the weight for the means covariance
    double firstWeight = parameters.lambda / (parameters.lambda + parameters.stateDimension) + (1 - pow(parameters.alpha, 2) + parameters.beta);
    weights(0) = firstWeight;
    //! 2*stateDimension weights because the "+1" is the first weight for the mean
    for (int i=1; i<parameters.sigmaPointAmount; i++) {
        double weight = 1 / (2 * (parameters.stateDimension + parameters.lambda));
        weights(i) = weight;
    }
    return weights;
}

/// normalizes the Angle so that it doesn't exceed the interval [-Pi,Pi]
/// \param angle double of the angle which will be normalizes in radians
/// \return double of the normalized angle in radians
double UnscentedKalmanFilter::normalizeAngle(const double angle, const double minimum, const double maximum )
{
    double returnAngle = angle;
    while (returnAngle > maximum)
        returnAngle -= 2*M_PI;
    while (returnAngle < minimum)
        returnAngle += 2*M_PI;
    return returnAngle;
}

/// Tests a Matrix for NAN and Inf values
/// \param testMatrix MatrixXd which is supposed to be tested
/// \return True if Matrix passes the test, False if it doesn't
bool UnscentedKalmanFilter::isMatrixValid(const MatrixXd testMatrix){
  for(int i = 0; i < testMatrix.rows(); i++){
    for(int j = 0; j < testMatrix.cols(); j++){
      if(std::isnan(testMatrix(i,j)) || std::isinf(testMatrix(i,j))){
        return false;
      }
    }
  }
  return true;
}

/// Tests a Vector for NAN and Inf values
/// \param testVector VectorXd which is supposed to be tested
/// \return True if Vector passes the test, False if it doesn't
bool UnscentedKalmanFilter::isVectorValid(const VectorXd testVector){
  for(int i = 0; i < testVector.size(); i++){
    if(std::isnan(testVector(i)) || std::isinf(testVector(i))){
      return false;
    }
  }
  return true;
}

/// Tests a Vector if it only contains 0 values
/// \param testVector VectorXd which is supposed to be tested
/// \return True if Vector passes the test, False if it doesn't
bool UnscentedKalmanFilter::doesVectorContainValues(const VectorXd testVector){
    for(int i = 0; i < testVector.size(); i++){
        if(testVector(i) != 0){
            return true;
        }
    }
    return false;
}

/// Tests a Matrix if it only contains 0 values
/// \param testMatrix MatrixXd which is supposed to be tested
/// \return True if Matrix passes the test, False if it doesn't
bool UnscentedKalmanFilter::doesMatrixContainValues(const MatrixXd testMatrix){
    for(int i = 0; i < testMatrix.rows(); i++) {
        for(int j = 0; j < testMatrix.cols(); j++) {
            if (testMatrix(i,j) != 0) {
                return true;
            }
        }
    }
    return false;
}