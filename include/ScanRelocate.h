/*******************
 * 重定位模块
 */

#ifndef  SCANRELOCATE_H
#define SCANRELOCATE_H

//#include<opencv2/core.hpp>
#include"System.h"

#include"TrackBlock.h"
#include<thread>
#include<unistd.h>
#include"vector"

using namespace std;
using namespace cv;


namespace LED_POSITION
{
class System;
class TrackBlock;


//ps: 重定位线程有三个个状态：查找状态、跟踪读取编码状态、休眠状态
class ScanRelocate
{
private://data
    System* mpSystem;//上一层的系统指针

    //扫描rg需要的缓存变量
    int mScanCntFlag;
    vector<Point2f> scanContourCenters_c;
    vector<double> scanSarea_c;

    //需要从sys那里得到的跟踪块信息
    vector<Point2f> mTBCenter;
    vector<int> mTBID;
    vector<int> mTBStatus;
    vector<Rect> mTBRect;

    //跟踪块参数
    int mTrackBlockWidth;
    bool fixedWidth;
    int mTrackBlockCodeLen=3;//bit

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

    //虚互斥锁
    std::mutex mMutexNO;

    
public:
    ScanRelocate(System *pSys,Mat frame0,bool fx,int tbw); 
    ~ScanRelocate();

    std::vector<TrackBlock> mTrackBlocks;
    void delateTrackBlock(int index);//把没要的TrackBlock 删掉（状态不是准备态的）

    //专用于本线程的扫描全图，检测R->G
    int scanFrame(int64 time_stamp);

    //线程主函数
    void Run();

    //updata
    void updataFromSys();

    //检查mTBStatus是否有losting的 有则返回ture
    bool checkTBStatusLOSTING();

    //差分 与sys一样
    Mat diffFrame(char nowColor,char lastColor);

    //恢复系统TB丢失状态
    void recoverBT();
};



}//LED_POSITION



#endif