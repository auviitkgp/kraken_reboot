#include "tcbt.h"

tCBT::tCBT(float dist_adj, float conf_adj)
{
    this->dist_adj = dist_adj;
    this->conf_adj = conf_adj;
}

void tCBT::init(cv::Mat &image, cv::Rect region){
    cbt.init(image, region, true);
    ratio = (float)region.height/(float)region.width;
    updateModel = false;
    state.clear();
    state.push_back(region.x);
    state.push_back(region.y);
    state.push_back(region.width);
}

void tCBT::correctState(std::vector<float> st){
    this->state = st;
    cbt.lastPosition.height = round(st[2]*ratio);
    cbt.lastPosition.width = round(st[2]);
    cbt.lastPosition.x = round(st[0] - st[2]/2.0);
    cbt.lastPosition.y = round(st[1] - (st[2]*ratio/2.0));
}

void tCBT::track(){
    float scale;
    double confidence = cbt.track(currentFrame, scale);
    float newWidth = state[2]*scale;

    this->stateUncertainty.clear();
    float penalityCBT = pow(dist_adj*fabs(state[0] - currentPredictRect[0])/((double)cbt.lastPosition.width),2)  +
                        pow(dist_adj*fabs(state[1] - currentPredictRect[1])/((double)cbt.lastPosition.height), 2);// +
                        //pow(dist_adj*fabs(state[2] - currentPredictRect[2])/((double)cbt.lastPosition.width),2);
    float uncertainty;
    if(confidence != 0){
        uncertainty = 1e-4*exp(-3.5*(conf_adj*confidence - penalityCBT));
        state.clear();
        state.push_back(round((float)cbt.lastPosition.x + (float)cbt.lastPosition.width/2.0));
        state.push_back(round((float)cbt.lastPosition.y + (float)cbt.lastPosition.height/2.0));
        state.push_back(newWidth);
    }else{
        uncertainty = 1;
    }

    stateUncertainty.push_back(uncertainty*5);
    stateUncertainty.push_back(uncertainty*5);
    stateUncertainty.push_back(uncertainty);
}

void tCBT::update(){
    cbt.update(currentFrame);
}

void tCBT::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tCBT::getRect(){
    cv::Rect rect;
    rect.x = round(cbt.lastPosition.x);
    rect.y = round(cbt.lastPosition.y);
    rect.width = round(cbt.lastPosition.width);
    rect.height = round(cbt.lastPosition.height);
    return rect;
}

