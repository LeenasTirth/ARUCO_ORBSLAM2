#include "ArucoMarker.h"
#include <unistd.h>
#include<mutex>
#include<set>

namespace ORB_SLAM2{
    ArucoMarker::ArucoMarker(const long unsigned int nId , bool bValid,float fSize,KeyFrame* pRefKF):
        mnId(nId),mpRefKF(pRefKF),mnFirstFrameid(pRefKF->mnId),mbValid(bValid),mfSize(fSize)
    {
        mvP3DM[0] = cv::Point3f(-fSize/2.0,fSize/2.0,0);
        mvP3DM[1] = cv::Point3f(fSize/2.0,fSize/2.0,0);
        mvP3DM[2] = cv::Point3f(fSize/2.0,-fSize/2.0,0);
        mvP3DM[3] = cv::Point3f(-fSize/2.0,-fSize/2.0,0);
    }

    void ArucoMarker::AddObservation(KeyFrame* pKF){
        unique_lock<mutex> lock(mMutexKF);
        mObservations.insert(pKF);
        nObs++;
    }

    cv::Mat ArucoMarker::GetWorldPos(){
        unique_lock<mutex> lock(mMutexPos);
        cv::Mat Tmw  = cv::Mat::eye(4,4,CV_32F);
        mRmw.copyTo(Tmw.rowRange(0,3).colRange(0,3));
        mtmw.copyTo(Tmw.rowRange(0,3).col(3));
        return Tmw;
    }

    void ArucoMarker::SetValidFlag(bool bValid){
        unique_lock<mutex> lock(mMutexValid);
        mbValid = bValid;
    }

    void ArucoMarker:: SetWorldPos(const cv::Mat& Rmw,const cv::Mat& tmw){
        unique_lock<mutex> lock(mMutexPos);
        Rmw.copyTo(mRmw);
        tmw.copyTo(mtmw);
    }

    KeyFrame* ArucoMarker::GetReferenceKeyFrame(){
        unique_lock<mutex> lock(mMutexKF);
        return mpRefKF;
    }

    bool ArucoMarker::GetValidFlag(){
        unique_lock<mutex> lock(mMutexValid);
        return mbValid;
    }

    int ArucoMarker::Observations(){
        unique_lock<mutex> lock(mMutexKF);
        return nObs;
    }

    void ArucoMarker::EraseObservation(KeyFrame* pKF){
        unique_lock<mutex> lock(mMutexKF);
        if(mObservations.count(pKF)){
            mObservations.erase(pKF);
            nObs--;
        }
    }

    bool ArucoMarker::IsInKeyFrame(KeyFrame*pKF){
        unique_lock<mutex> lock(mMutexKF);
        return mObservations.count(pKF);
    }

}