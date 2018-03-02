#include "kfebtracker.h"

KFebTracker::KFebTracker()
{
    asms = tASMS(ajuste, 0.90);
    kcf = tKCF(ajuste, 1.15);
    cbt = tCBT(ajuste, 0.45);
    vdp = tVDP(ajuste, 0.60);
    ncc = tncc(ajuste, 0.7);
    gmm = tGMM(ajuste,0.9);
    //gmm=tgmm(adjuste,1.9)
}

/* Initiation Parameters
 * A = ASMS
 * K = KCF
 * C = CBT
 * V = VDP
 * N = NCC
 *
 * (Send string "AKN" to initialize the tracker with ASMS, KCF and NCC)
 */
void KFebTracker::init(std::string initPar){
    trackers.clear();
    for(int i = 0; i < (int)initPar.length(); i++){
        switch (initPar[i]) {
        case 'A':
            trackers.push_back(&asms);
            break;
        case 'K':
            trackers.push_back(&kcf);
            break;
        case 'C':
            trackers.push_back(&cbt);
            break;
        case 'V':
            trackers.push_back(&vdp);
            break;
        case 'N':
            trackers.push_back(&ncc);
            break;
        case 'G':
              trackers.push_back(&gmm);
        default:
            break;
        }
    }
}

void KFebTracker::initTrackers(cv::Mat image, cv::Rect region){
    // Inicialization
    for(unsigned int i = 0; i < trackers.size(); i++){
        trackers[i]->init(image, region);
    }

    // Alocate KFEBT
    fusion = KFEBT(9, 3*trackers.size(), 0, 0.05, region);
    //kf
}

cv::Rect KFebTracker::track(cv::Mat image){
    // Start trackers

    //suscribe kf
    for(unsigned int i = 0; i < trackers.size(); i++){
        trackers[i]->newFrame(image, fusion.getPrediction());
        count=0;
        trackers[i]->start();

      }
    //if catch_val == 0 ..then don;t do anything no updates to kf
    // Wait and get results

        uncertainty.clear();
        trackersResults.clear();
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->wait();
            uncertainty.insert(uncertainty.end(), trackers[i]->stateUncertainty.begin(), trackers[i]->stateUncertainty.end());
            trackersResults.insert(trackersResults.end(), trackers[i]->state.begin(), trackers[i]->state.end());

        }

    //if (trackers[1]->state[0]==0 && trackers[1]->state[1]==0 &&  trackers[1]->state[1]==0 )
  //          {
  //            count++;
  //          std::cout<<count<<" 1"<<std::endl;
  //        }


        if(count)
        {
          for(unsigned int i = 0; i < trackers.size(); i++){
              trackers[i]->start();
          }

          // Wait trackers update process
          for(unsigned int i = 0; i < trackers.size(); i++){
              trackers[i]->wait();
          }
          cv::Rect a(10,10,10,10);
          return a;

        }

        // Correct the KF
        fusion.correct(trackersResults, uncertainty);

        // Model Update
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->start();
        }

        // Wait trackers update process
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->wait();
        }

        // Feedback
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->correctState(fusion.getFusion());
        }

        // Predict next frame state
        fusion.predict();

        //Report result
        return fusion.getResult();


}
