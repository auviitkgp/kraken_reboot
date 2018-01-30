#ifndef KFEBT_H
#define KFEBT_H

#include <opencv2/opencv.hpp>
#include <vector>

class KFEBT
{
public:
    KFEBT();
    KFEBT(int nStates, int nMeasurements, int nInputs, double dt, cv::Rect initialState);
    void predict();
    void correct(std::vector<float> measures, std::vector<float> Uncertainty);
    std::vector<float> getFusion();
    cv::Rect getResult();
    std::vector<float> getPrediction();
    void setProcessCov(float cov);

private:
    cv::KalmanFilter KF;
    cv::Mat estimated;
    cv::Mat corrected;
    cv::Mat KFMeasures;
    float ratio;

};

#endif // KFEBT_H
