#ifndef  SCANRELOCATE_H
#define SCANRELOCATE_H

#include<opencv2/core.hpp>
#include"TrackBlock.h"
#include<thread>
#include"System.h"

namespace LED_POSITION
{
class System;


//ps: 重定位线程有三个个状态：查找状态、跟踪读取编码状态、休眠状态
class ScanRelocate   
{
private://data
    System* mpSystem;//上一层的系统指针

    
public:
    ScanRelocate(System *pSys);
    ~ScanRelocate();

    //线程主函数
    void Run();
};




}//LED_POSITION



#endif