# ORBSLAM2自学文档

## 自定义oop类型介绍



### System类

#### 成员变量

##### 公有

无，仅定义了一个枚举类型**eSensor**。其中0表示单目，1表示双目，2表示RGBD。

##### 私有

| 名称                         | 类型                       | 用途                           | 是否是自定义的类型                |
| ---------------------------- | -------------------------- | ------------------------------ | --------------------------------- |
| mSensor                      | eSensor                    | 表示该程序使用的是哪种相机类型 | 是                                |
| mpVocabulary                 | ORBVocabulary*             | 指向ORB字典的指针              | 是                                |
| mpKeyFrameDatabase           | KeyFrameDatabase*          | 指向关键帧数据库的指针         | 是                                |
| mpMap                        | Map*                       | 指向地图数据库的指针           | 是                                |
| mpTracker                    | Tracking*                  | 指向Tracking的指针             | 是                                |
| mpLocalMapper                | LocalMapping*              | 指向LocalMapping的指针         | 是                                |
| mpLoopCloser                 | LoopClosing*               | 指向LoopClosing的指针          | 是                                |
| mpViewer                     | Viewer*                    | 指向可视化查看器的指针         | 是                                |
| mptLocalMapping              | std::thread*               | 指向局部建图线程的指针         | 不是                              |
| mptLoopClosing               | std::thread*               | 指向回环检测线程的指针         | 不是                              |
| mptViewer                    | std::thread*               | 指向可视化查看器线程的指针     | 不是                              |
| mMutexReset                  | std::mutex                 | 不确定。用于复位的             | 不是                              |
| mbReset                      | bool                       | 不确定。用于复位的             | 不是                              |
| mMutexMode                   | std::mutex                 | 不确定。用于改模式的           | 不是                              |
| mbActivateLocalizationMode   | bool                       | 启动定位模式                   | 不是                              |
| mbDeactivateLocalizationMode | bool                       | 关闭定位模式                   | 不是                              |
| mTrackingState               | int                        | 不确定。                       | 不是                              |
| mTrackedMapPoints            | std::vector<MapPoint*>     | 维护追踪到的地图点             | 不是（其中的MapPoint*是自定义的） |
| mTrackedKeyPointsUn          | std::vector\<cv::KeyPoint> | 维护追踪到的去畸变后的特征点   | 不是                              |
| mMutexState                  | std::mutex                 | 不确定                         | 不是                              |

#### 成员函数

##### 公有

```c++
System(const string &strVocFile,const string &strSettingsFile,const eSensor sensor,const bool bUseViewer = true);//构造函数，用来初始化整个系统。在这个构造函数中会启动三大线程并使用指针互相指向，为三大线程建立联系。
```



```c++
cv::Mat TrackStereo(const cv::Mat &imLeft,const cv::Mat &imRight,const double &timestamp);//为双目相机设计的运动追踪tracking接口。传入左右目彩色图像和时间戳。
```



```c++
cv::Mat TrackRGBD(const cv::Mat &im,const cv::Mat &depthmap,const double &timestamp);//为RGBD相机设计的运动追踪Tracking接口。传入一张彩色图像、一张深度图和时间戳。
```



```c++
cv::Mat TrackMonocular(const cv::Mat &im,const double &timestamp);//为单目相机设计的运动追踪Tracking接口。传入一张彩色图像和时间戳。
```



```c++
void ActivateLocalizationMode();//只开启定位模式，此时只有运动追踪部分在工作。
```



```c++
void DeactivateLocalizationMode();//关闭定位模式，开启整个slam建图与定位。
```



```c++
bool MapChanged();//与上次调用本函数相比，地图是否发生较大变化。
```



```c++
void Reset();//复位系统。
```



```c++
void Shutdown();//关闭系统。结束所有线程。
```



```c++
void SaveTrajectoryTUM(const string &filename);//以TUM格式保存相机的运动轨迹，这个函数将会在Shutdown函数中被首先调用。
```



```c++
void SaveKeyFrameTrajectoryTUM(const string &filename);//以TUM格式保存关键帧位姿
```



```c++
void SaveTrajectoryKITTI(const string &filename);//以KITTI格式保存相机的运行轨迹
```



```c++
int GetTrackingState();//获取最近的运动追踪状态、地图点追踪状态、特征点追踪状态
```



```c++
std::vector<MapPoint*> GetTrackedMapPoints();//获得追踪到的地图点。
std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();//获得追踪到的特征点。
```



### Tracking类

#### 成员变量

##### 公有

| 名称                 | 类型                      | 用途                                                         | 是否自定义                |
| -------------------- | ------------------------- | ------------------------------------------------------------ | ------------------------- |
| eTrackingState       | enum                      | 跟踪状态的类型                                               |                           |
| mState               | eTrackingState            | 当前帧的跟踪状态                                             | 是                        |
| mLastProcessedState  | eTrackingState            | 上一帧的跟踪状态                                             | 是                        |
| mSensor              | int                       | 传感器类型                                                   | 不是                      |
| mCurrentFrame        | Frame                     | 当前正在处理的这一帧                                         | 是                        |
| mImGray              | cv::Mat                   | 当前帧的灰度图像                                             | 不是                      |
| mvIniLastMatches     | std::vector\<int>         | 不确定。初始化阶段中,当前帧中的特征点和参考帧中的特征点的匹配关系 | 不是                      |
| mvIniMatches         | std::vector\<int>         | 不确定。跟踪初始化时前两帧之间的匹配                         | 不是                      |
| mvbPrevMatched       | std::vector\<cv::Point2f> | 不确定。在初始化的过程中,保存参考帧中的特征点                | 不是                      |
| mvIniP3D             | std::vector\<cv::Point3f> | 不确定。初始化过程中匹配后进行三角化得到的空间点             | 不是                      |
| mInitialFrame        | Frame                     | 初始化过程中的参考帧                                         | 是                        |
| mlRelativeFramePoses | list\<cv::Mat>            | 所有的参考关键帧的位姿                                       | 不是                      |
| mlpReferences        | list<KeyFrame*>           | 参考关键帧                                                   | 不是（KeyFrame*是自定义） |
| mlFrameTimes         | list\<double>             | 所有时间戳                                                   | 不是                      |
| mlbLost              | list\<bool>               | 是否跟丢的标志                                               | 不是                      |
| mbOnlyTracking       | bool                      | 标记当前系统是处于SLAM状态还是纯定位状态                     | 不是                      |

##### 私有（protected）

| 名称 | 类型 | 用途 | 是否自定义 |
| ---- | ---- | ---- | ---------- |
|      |      |      |            |



