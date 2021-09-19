# 新增的自定义类型设计

## ArucoMarker

**成员变量**

```c++
public:
	long unsigned int mnId;//这个二维码标记的id
	KeyFrame* mnFirstKF;//第一次发现这个标记的关键帧，也就是参考关键帧
	const long int mnFirstFrameid;//创建该标记的帧id
	int nObs;//这个标记被观测到的次数
protected:
	cv::Mat mRmw;//世界坐标系下到marker坐标系的旋转矩阵
	cv::Mat mtmw;//世界坐标系到marker坐标系的位移变换
	float mfSize;//marker的尺寸大小，即宽的长度
	Map* mpMap;//所属地图
	std::vector<cv::Point3f> mvP3DM;//marker坐标系上的四个角点的3D位置，顺时针从左上角开始
	bool mbValid;//该标记是否被有效的估计出了位姿
	std::set<KeyFrame*> mObservations;//哪些关键帧观察到了这个标记

```

**成员函数**

```c++
public: 
	ArucoMarker();//构造函数
	void SetWorldPos(const cv::Mat& Rmw,const cv::Mat& tmw);//为二维码设置世界坐标系位姿
	cv::Mat GetWorldPos();//返回世界坐标系位姿[R,t;0,1]
	KeyFrame*GetReferenceKeyFrame();//获取生成当前二维码的参考关键帧.
	void SetValidFlag(bool valid);//设置标记是否有效
	bool GetValidFlag();//获取标记是否有效
	int Observations();//获取被观测到的次数
	void AddObservation(KeyFrame* pKF);//新增观察到该标记的关键帧
	void EraseObservation(KeyFrame* pKF);//移除掉该标记被某个关键帧观测到的记录
	bool IsInKeyFrame(KeyFrame* pKF);//查看某个关键帧中是否观测到了该标记
private:

```



## ArucoMarkerDatabase

**成员变量**

```c++
public:
	
protected:
```

**成员函数**

```c++
```



# 其他类型的修改

## Frame

**成员变量**

```c++
public:
	std::map<int,std::pair<cv::Mat,cv::Mat>>ambiguity; //pair的第二个元素为空则表示无歧义，否则为有歧义。
	
```

**成员函数**

```c++
public:
	void ExtractMarker(const cv::Mat &im); //用于从当前图像中检测到所有出现了的marker，并且计算出相对位姿变换保存到ambiguity中。
```

## Tracking

**成员变量**

```c++
protected:
	float mfMarkerSize;// 用于从配置文件中获取标记的真实尺寸
```

