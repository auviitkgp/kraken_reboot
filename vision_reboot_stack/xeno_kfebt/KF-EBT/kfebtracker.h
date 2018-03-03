#ifndef KFEBTRACKER_H
#define KFEBTRACKER_H

#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "trackers/tcbt.h"
#include "trackers/tmosse.h"
#include "trackers/tvdp.h"
#include "trackers/tncc.h"
#include "trackers/tgmm.h"
#include "kfebt.h"

class KFebTracker
{
public:
    KFebTracker();
    void init(std::string initPar);
    void initTrackers(cv::Mat image, cv::Rect region);
    cv::Rect track(cv::Mat image);

private:
    float ajuste = 0.15;
    int count=0;
    // Trackers
    tASMS asms;
    tKCF kcf;
    tCBT cbt;
    tVDP vdp;
    tncc ncc;
    tGMM gmm;

    std::vector<BTracker*> trackers;

    // Kalman Filter
    KFEBT fusion;

    std::vector<float> uncertainty, trackersResults;
};

#endif // KFEBTRACKER_H
