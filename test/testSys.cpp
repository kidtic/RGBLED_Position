#include"System.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include<time.h>

using namespace cv;
using namespace std;


int main(int argc, char const *argv[])
{
     VideoCapture cap(argv[1]);

    if(!cap.isOpened())
    {
        printf("无法打开视频");
    }
    Mat frame;
    cap.read(frame);
    LED_POSITION::System ledTrack(frame,2);
    int ff=0;
    namedWindow("src",0);

    //主循环
    while (cap.read(frame))
    {
        //printf("track:\n");
        time_t start,stop;
        double totaltime;
        start=getTickCount();
        //--------------------主入口
        if(ff==0)
            ff=ledTrack.Init(frame);
        else if(ff==1)
        {
            ledTrack.position(frame);
        }
        
        //----------------画图
        for (size_t i = 0; i < ledTrack.mTrackBlocks.size(); i++)
        {
            Point2f pc=ledTrack.mTrackBlocks[i].getCenter();
            //circle(frame,pc,15,Scalar(255,255,0),3);
            rectangle(frame,ledTrack.mTrackBlocks[i].getTrackRect(),Scalar(255,255,0),1);
            int idd=ledTrack.mTrackBlocks[i].getcodeID();
            putText(frame, to_string(idd), pc+Point2f(0,-10), FONT_HERSHEY_COMPLEX, 1.0, Scalar(100, 200, 200), 2);
        }
        stop = getTickCount();
        //printf("Use Time:%ldms\n",(stop-start)/1000000);   
        
        imshow("src",frame);
        waitKey(1);
        
        
    }
}