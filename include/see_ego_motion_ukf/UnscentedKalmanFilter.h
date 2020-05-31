
#ifndef SEE_EGO_MOTION_CPP_UNSCENTEDKALMANFILTER_H
#define SEE_EGO_MOTION_CPP_UNSCENTEDKALMANFILTER_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*!
 *struct containing all adjustable parameters of the unscented kalman filter
 * "std" -> stands for standard Deviation
 */
typedef struct EME_ParameterList {
    int stateDimension;        //! state dimensions
    int sigmaPointAmount;    //! amount of sigma points that have to be generated
    double lambda;          //! scale parameter resulting out of alpha sqrt(alpha)*(stateDimension+kappa)-statedimension
    double alpha;           //! scale parameter in the range of (0,1]
    double beta;            //! used to incorporate prior knowledge of x's distribution -> 2 is optimal for gaussians
    int kappa;              //! scale parameter -> usually 0 or 3-L
    double std_YawRate;        //! process noise standard deviation yaw acceleration
    double std_Acceleration;//! process noise standard deviation longitudinal acceleration
} EME_ParamList;


/*!
 *Implementation of the unscented kalman filter
 */
class UnscentedKalmanFilter {
public:
    /*!
     * Constructor
     */
    UnscentedKalmanFilter();

    /*!
     * Destructor
     */
    ~UnscentedKalmanFilter();

    /// Getter for the state vector
    /// \return VectorXd of the states vector
    VectorXd getState();

    /// Getter for the state covariance matrix
    /// \return MatrixXd of the state covariance
    MatrixXd getStateCovariance();

    /// Getter for the matrix that contains the predicted sigma points
    /// \return MatrixXd of the sigma points
    MatrixXd getPredictedSigmaPoints();

    /// Getter for the initialization status of the unscented kalman filter
    /// \return true if initialized; False if not
    bool isInitialized();

    /// Initializes the unscented kalman filter
    /// \return true if initialized; False if not
    bool initialize(VectorXd firstState);

    /// Predict step of the Kalman filter
    /// Basically Unscented Transforms the old state to a new predicted mean and covariance
    /// \param deltaTime elapsed time between states
    /// \return predicted sigma points
    MatrixXd predict(const double deltaTime);

    /// Updates the state with the transformed measurements
    /// \param measurements VectorXd containing the original measurements
    /// \param measurementNoise MatrixXd containing the noise matrix of the Sensor
    /// \param measurementSigmaPoints MatrixXd containing the sigma points from the measurements
    /// \param predictedSigmaPoints MatrixXd containing the predicted sigma points
    /// \param measurementDimension int of the measurement vector dimension
    /// \param indexOfMeasuredYaw int of the index of the measured vectors yaw
    ///                          (use any value below 0 or above 5 to indicate that there is no yaw)
    void update(VectorXd &measurements, MatrixXd &measurementNoise,
                MatrixXd &measurementSigmaPoints, MatrixXd predictedSigmaPoints,
                const int measurementDimension, const int indexOfMeasuredYaw);

    /// Tests a Matrix for NAN and Inf values
    /// \param testMatrix MatrixXd which is supposed to be tested
    /// \return True if Matrix passes the test, False if it doesn't
    bool isMatrixValid(const MatrixXd testMatrix);

    /// Tests a Vector for NAN and Inf values
    /// \param testVector VectorXd which is supposed to be tested
    /// \return True if Vector passes the test, False if it doesn't
    bool isVectorValid(const VectorXd testVector);

    /// Tests a Vector if it only contains 0 values
    /// \param testVector VectorXd which is supposed to be tested
    /// \return True if Vector passes the test, False if it doesn't
    bool doesVectorContainValues(const VectorXd testVector);

    /// Tests a Matrix if it only contains 0 values
    /// \param testMatrix MatrixXd which is supposed to be tested
    /// \return True if Matrix passes the test, False if it doesn't
    bool doesMatrixContainValues(const MatrixXd testMatrix);

private:
    bool initialized;               //! boolean stating the initialization status -> false per default until initialized
    VectorXd state;                 //! state vector with values: [xPosition, yPosition, yaw(Psi), yaw rate(dPsi), velocity(v), acceleration(a)]
    MatrixXd stateCovariance;       //! covariance matrix of the state
    MatrixXd predictedSigmaPoints;  //! matrix containing the predicted sigma points
    VectorXd stateWeights;          //! weight vector for the sigma points of the size sigmaPointAmount
    VectorXd stateCovarianceWeights;//! weight vector for covariance of the sigma points of the size sigmaPointAmount
    EME_ParameterList parameters;   //! list of all relevant UKF parameters

    /// Generates the set of sigma points out of the given state vector and covariance matrix
    /// \param initialState     the current state which the point will be centered around
    /// \param initialStateCovariance state covariance of the state for calculation of the points
    /// \return MatrixXd of sigma points
    bool Initialize(VectorXd initialState, MatrixXd initialStateCovariance);

    /// Generates the set of sigma points out of the given state vector and covariance matrix
    /// \param initialState     the current state which the point will be centered around
    /// \param initialStateCovariance state covariance of the state for calculation of the points
    /// \return MatrixXd of sigma points
    MatrixXd generateSigmaPoints(VectorXd initialState, MatrixXd initialStateCovariance);

    /// Predict all Sigma Points through the CTRA motion model without taking Noise into account
    /// \param sigmaPoints MatrixXd of the generated set of sigma points
    /// \param deltaTime time interval between the old and the "to predict" state
    /// \return MatrixXd of the predicted SigmaPoints
    MatrixXd sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime);

    /// Calculater the mean state vector using the sigma points and old state
    /// \param state base state vector for the prediction
    /// \param predictedSigmaPoints MatrixXd of the predicted sigma point set
    /// \return VectorXd of predicted state vector
    VectorXd calculateState(VectorXd &state, MatrixXd &predictedSigmaPoints);

    /// Predict the covariance matrix
    /// \param stateCovariance base covariance matrix for the prediction
    /// \param predictedSigmaPoints MatrixXd which contains the set of predicted sigma points
    /// \param predictedState VectorXd of the predicted state vector
    /// \param processNoise MatrixXd of the process noise
    /// \return MatrixXd of the predicted state covariance
    MatrixXd predictCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState,
                               MatrixXd &processNoise, int indexOfMeasuredYaw);

    /// Calculates the Process Noise of the new deltatime
    /// \param deltaTime elapsed time between states
    /// \param yaw predicted yaw angle of the predicted state vector
    /// \return MatrixXd of the process noise matrix
    MatrixXd calculateProcessNoise(const double &deltaTime, const double &yaw);

    /// Sets the weights for the state of the Uncented Kalman Filter.
    /// \return VectorXd of the states weights of size sigmaPointAmount
    VectorXd setStateWeights();

    /// Sets the weights for the states covariance of the Unscented Kalman Filter
    /// \return VectorXd of the states covariance weights of size sigmaPointAmount
    VectorXd setCovarianceWeights();

    /// normalizes the Angle so that it doesn't exceed 2*Pi
    /// \param angle double of the angle which will be normalizes
    /// \return double of the normalized angle
    double normalizeAngle(const double angle, const double minimum, const double maximum);

};

#endif //SEE_EGO_MOTION_CPP_UNSCENTEDKALMANFILTER_H
