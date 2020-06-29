#ifndef  TRACKBLOCK_H
#define TRACKBLOCK_H

#include<vector>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include<iostream>
#include<mutex>
#include"System.h"



using namespace cv;
using namespace std;

namespace LED_POSITION
{
class System;
//跟踪块
class TrackBlock
{
public:
    //跟踪块的状态
    enum eTrackStatus{
        OK=0,
        LOSTING=1,
        PREPARE=2,//系统初始化后，还没有读取code，正在读取code状态
        SUSPECTED_LOST=3,//疑似丢失
    };

    //LED编码的读取状态
    enum eCodeStatus{
        FINISH=0,
        READING=1
    };
    
protected://data
    //块区域，rio感兴趣区域，是相对于System的原始图像
    Rect rect;
    Rect rect_last;//上一帧的块区域
    Rect rect_pre;//上上一帧的块区域
    Size srcImgSize;

    //中心点，该跟踪块所跟踪的LED的中心点，是相对于原始图像的。
    Point2f ledCenter;
    //std::mutex mMutexCenter;
    double mContourArea;//轮廓面积
    Point2f mVelocity;//上一帧的运动速度
    //决定mTrackBlockWidth的大小，根据轮廓面积来给出TrackBlockWidth
    double mTrackBlockWidth_Sarea=13;  

    //块图像
    Mat BlockImg;
    Mat BlockImg_last;//上一帧的图像块
    Mat BlockImg_pre;//上上一帧的图像块

    //此时的跟踪状态
    eTrackStatus TrackStatus;
    int IDNoMatchNUM=0;//如果多次检查ID有错误，则直接丢失
    //std::mutex mMutexTrackStatus;


    //编码信息
    int codeLength;//编码的长度
    eCodeStatus codeStatus;//编码状态
    std::vector<int> codeCashe;//编码缓存队列，每3帧读取一位
    int codeID;//编码ID 
    //std::mutex mMutexCodeID;

    int64 startread_time;  //R->G跳变 的时间，以这个时间为起始时间。

    //---------读取编码信息需要的变量
    float fps=30;
    //一个颜色保持时间为多长（ms）
    float dt=200.0; 
    //在startread_ftime=0的时候，应该是RG->Data的时间，初始化的时候应该是
    //负数：15-dt  (ms)
    float startread_ftime;
    //计数指针，在检测到R->G后，读取了多少信息，不超过codeLength
    int mpReadCodeCNT;
    //R-G跳变信息，codeStatus=finish的时候，rgtbStatus='S',
    //当检测到R的时候rgtbStatus='R',然后在检测到G的时候rgtbStatus='G'
    char rgtbStatus;

    //颜色常量
    Scalar lower_blue;
    Scalar upper_blue;
    Scalar lower_green;
    Scalar upper_green;
    Scalar lower_red0;
    Scalar upper_red0;
    Scalar lower_red1;
    Scalar upper_red1;


    //System *mpSystem;
    std::mutex* pSysMutex; 
    


public:
    TrackBlock(std::mutex *pMutex,Mat srcinput,Point2f initPoint,int rectwidth,int mcodeLength,int64 time_stamp);
    TrackBlock(Mat srcinput,Point2f initPoint,int rectwidth,int mcodeLength,int64 time_stamp);
    ~TrackBlock();
    

    //跟踪函数
    //输入当前帧的原始图像，会根据块区域来识别新的LED点在哪里
    //同时还会读取LED编码信息与上一帧的编码信息进行验证
    eTrackStatus track(Mat srcinput,int64 time_stamp);

    //卡尔曼filter
    bool Kalmanfilter(Mat srcinput,char& rgbcode);

    //查找颜色轮廓，返回RGB的轮廓点
    //inputImg：输入图像
    //rgb：输出的rgb轮廓点
    //Sarea：rgb对应轮廓点的轮廓面积
    void findContourCenterRGB(Mat inputImg ,vector<Point2f> &r,vector<Point2f> &g,vector<Point2f> &b,vector<double> &Sarea);
    void findContourCenterGB(Mat inputImg ,vector<Point2f> &g,vector<Point2f> &b,vector<double> &Sarea);
    
    //返回rgb轮廓中，与中心点最接近的向量，并且该向量要满足运动速度校验
    //rgb：输入，findContourCenterRGB找出来的轮廓点
    //Sarea：输入，rgb对应的轮廓点的轮廓面积
    //OutputminVec：输出变化向量
    //OutputSarea：该点的面积
    char findminVecRGB(vector<Point2f> r,vector<Point2f> g,vector<Point2f> b,vector<double> Sarea,Vec2f &OutputminVec,double &OutputSarea);

    Point2f getCenter();
    int getcodeID();
    Rect getTrackRect();
    eTrackStatus getStatus();

    //设置数据
    void setCenter(Point2f p);
    void setcodeID(int id);
    void setStatus(eTrackStatus status);
    void setStartTime(int64 sttime);
    bool setTrackRect(Rect rt);
    //加锁
    void mutexLock();
    void mutexUnLock();



};



}//namespace LED_POSITION

#endif