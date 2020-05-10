#ifndef  SYSTEM_H
#define SYSTEM_H

#include<opencv2/core.hpp>
#include"TrackBlock.h"
#include<thread>
#include"ScanRelocate.h"
#include<vector>
#include<Cluster.h>
#include <mutex>
#include <map>

using namespace std;

namespace LED_POSITION
{

class TrackBlock;
class ScanRelocate;
class Cluster;

class System
{
public:
    enum eSysStatus{
        INIT=0,
        POSITION=1
    };
    enum drawType{
        CIRCLE=0,
        BLOCK=1
    };
private://data

    //系统状态
    eSysStatus sysStatus=INIT;//初始化状态和定位状态
    int lostingTB_Num;//丢失的跟踪块数目

    //新的一个线程，全图扫描，重新找寻丢失的跟踪块
    std::thread* mptScanRelocate;
    ScanRelocate* mpScanRelocater;
    std::mutex mMutexInputFlag;
    bool mInputScanRelocateFlag=false;

    //用于初始化的时候状态转移
    int initCntFlag;
    vector<Point2f> initContourCenters_c;  //用于初始化的时候存储轮廓点缓存
    vector<double> initSarea_c;   //用于初始化的时候存储轮廓点面积缓存

    //跟踪块参数
    int mTrackBlockWidth;
    int mTrackBlockCodeLen=3;//bit
    bool fixedWidth=false; //是否采用固定TB大小

    //颜色常量
    Scalar lower_blue;
    Scalar upper_blue;
    Scalar lower_green;
    Scalar upper_green;
    Scalar lower_red0;
    Scalar upper_red0;
    Scalar lower_red1;
    Scalar upper_red1;

    //决定mTrackBlockWidth的大小，根据轮廓面积来给出TrackBlockWidth
    double mTrackBlockWidth_Sarea=13;  
    

    //原始图像
    std::mutex mMutexFrame;
    Mat frame,frame_last,frame_pre;
    //低分辨率图像
    Mat re_frame,re_frame_last,re_frame_pre;

    //更改分辨率size
    Size lowSize;
    double sizek;//-------高分辨率原始图像宽/低分辨率图像宽  >0



public://data
    //跟踪块，在初始化之后，push进，之后不再增加或减少
    std::vector<TrackBlock> mTrackBlocks;
    std::mutex mMutexTrackBlocks;

     //聚类器
    Cluster mCluster;


public:

    /*
    *@brief：构造的时候提供第一帧的图像
    *@param:frame0:提供第一帧图像
            resizek:提供缩放比
            fixedw:是否固定跟踪框大小
            tbWidth:跟踪块大小 
    */
    System(Mat frame0,double resizek,bool fixedw,int tbWidth=40);
    ~System();

    //初始化函数
    //注意这个函数需要重复使用，要一直调用到返回值为1未知
    //返回初始化是否完成：0未完成；1已完成；-1初始化失败
    eSysStatus Init(Mat frameInput);

    //初始化函数
    //输入id号 直接生成TB，不用扫描
    eSysStatus Init(Mat frameInput,vector<int> ids);

    //系统主程序
    void position(Mat frameInput);

    //系统运行函数系统总架构
    void run(Mat frameInput);

    //画图
    void drawObject(Mat& frameInput,drawType t);


    //hsv差分，全局图像颜色差分，默认采用低分辨率图像，所以输出图像也是低分辨率的
    //nowColor：当前帧的颜色  {'R','G','B'}
    //lastColor：上一帧的颜色
    Mat diffFrame(char nowColor,char lastColor);

    //查找满足要求的轮廓的中心点
    //inputImg:输入图像
    //Sarea:轮廓面积要大于这个数
    vector<Point2f> findContourCenter(Mat inputImg ,vector<double> &Sarea);

    //返回一些参数
    double getTrackBlockWidth_Sarea();
    int getTrackBlockCodeLen();
    Size getLowSize();

    //返回led中心点，只返回ok与sub状态的点
    map<int,Point2f> getLEDPoint();

    //需要互斥锁
    //返回图像 times=0 表示当前帧
    Mat getFrame(int times);

    //设置重新定位线程的输入输出标志位
    void setInputScanRelocateFlag(bool flg);
    bool getInputScanRelocateFlag();

    

};





}//namespace LED_POSITION
#endif 