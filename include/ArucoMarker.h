#ifndef ARUCOMARKER_H
#define ARUCOMARKER_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include<set>
#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2{
class KeyFrame;
class Map;
class Frame;

class ArucoMarker{
    //public function
    public:
        ArucoMarker(const long unsigned int nId , bool bValid,float fSize,KeyFrame* pRefKF);

        //set the world pos for marker.
        void SetWorldPos(const cv::Mat& Rmw,const cv::Mat& tmw);//为二维码设置世界坐标系位姿

        //get the world pos of the marker.[R,t;0,1]
        cv::Mat GetWorldPos();//返回marker的世界坐标系

        //get the reference keyframe of the marker.
        KeyFrame* GetReferenceKeyFrame();//获取生成当前二维码的参考关键帧.

        //set the valid bit to marker.
        void SetValidFlag(bool bValid);//设置标记是否有效

        //get the valid bit of the marker.
        bool GetValidFlag();//获取标记是否有效

        //get the number of times observed
        int Observations();//获取被观测到的次数

        //add a new keyframe that observes the marker.
        void AddObservation(KeyFrame* pKF);//新增观察到该标记的关键帧

        // erase the observation relationship in which the keyframe observes the marker.
        void EraseObservation(KeyFrame* pKF);//移除掉该标记被某个关键帧观测到的记录

        //see if the marker is observed in a keyframe
        bool IsInKeyFrame(KeyFrame* pKF);//查看某个关键帧中是否观测到了该标记
    public:
        const long unsigned int mnId;//the id of the marker.
        KeyFrame* mpRefKF;// the reference keyframe.
        const long int mnFirstFrameid;;//the id of the reference keyframe.
        int nObs;//the number of times the marker is observed.

    protected:
        cv::Mat mRmw; //Rotation :world -> marker.
        cv::Mat mtmw; // translation:world -> marker.
        float mfSize; // the real size of marker.
        Map* mpMap; // point to Map.
        std::vector<cv::Point3f> mvP3DM; //3D pos of 4 points in the marker corner. Clockwise and the first is top left corner.
        bool mbValid; // valid bit of pos.
        std::set<KeyFrame*> mObservations; // the set of keyframes that observes the marker.
        std::mutex mMutexKF;//the lock of keyframe.
        std::mutex mMutexPos;
        std::mutex mMutexValid;


};

}//namespace ORB_SLAM2

#endif