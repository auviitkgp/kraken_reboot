#include "tgmm.h"

tGMM::tGMM(float dist_adj, float conf_adj)
{
    this->dist_adj = dist_adj;
    this->conf_adj = conf_adj;
}

void tGMM::init(cv::Mat& image, cv::Rect region){
    gmm.init(image,region);
    ratio = (float)region.height/(float)region.width;
    r_gmm=region;
    updateModel = false;

}

void tGMM::correctState(std::vector<float> st){
    this->state = st;
    r_gmm.height = st[2]*ratio;
    r_gmm.width = st[2];
    r_gmm.x = st[0] - r_gmm.width/2;
    r_gmm.y = st[1] - r_gmm.height/2;
}
//this needs to modified properly
void tGMM::track(){


    r_gmm=gmm.track(currentFrame,r_gmm);

    // write state
    state.clear();
    state.push_back(r_gmm.x);
    state.push_back(r_gmm.y);
    state.push_back(r_gmm.width);
      ///need to change this part
    this->stateUncertainty.clear();
    float penalityGMM = pow(dist_adj*fabs(state[0] - currentPredictRect[0])/((double)r_gmm.width),2)  +
                         pow(dist_adj*fabs(state[1] - currentPredictRect[1])/((double)r_gmm.height), 2);// +
                         //pow(dist_adj*fabs(state[2] - currentPredictRect[2])/(double)asms.lastPosition.width,2);
        ///need to decide confidenceGMM kya karu iska
    float uncertainty = 1e-5;
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
}

void tGMM::update(){

}

void tGMM::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tGMM::getRect(){
    return r_gmm;
}
///why oo why ratio is constatnt
