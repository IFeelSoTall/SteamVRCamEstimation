class KalmanFilter
{
private:
    double X0; // predicted state
    double P0; // predicted covariance
    double F;  // factor of real value to previous real value
    double Q;  // measurement noise
    double H;  // factor of measured value to real value
    double R;  // environment noise
public:
    double State, Covariance;
    KalmanFilter(double q, double r, double f, double h);
    void SetState(double state, double covariance);
    void Correct(double data);
};
KalmanFilter::KalmanFilter(double q, double r, double f, double h)
{
    this->Q = q;
    this->R = r;
    this->F = f;
    this->H = h;
}

void KalmanFilter::SetState(double state, double covariance)
{
    this->State = state;
    this->Covariance = covariance;
}

void KalmanFilter::Correct(double data)
{
    //time update - prediction
    X0 = F * State;
    P0 = F * Covariance * F + Q;
    //measurement update - correction
    double K = H * P0 / (H * P0 * H + R);
    State = X0 + K * (data - H * X0);
    Covariance = (1 - K * H) * P0;
}
