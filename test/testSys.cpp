#include "System.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <time.h>

using namespace cv;
using namespace std;


int main(int argc, char const *argv[])
{
    string rtspstr="rtsp://admin:zou133zzq@192.168.123.110:554/cam/realmonitor?channel=1&amp;subtype=0&amp;unicast=true&amp;proto=Onvif";
    string redstr="/home/kk/dataset/led/aimibot.mp4";
    VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,1920);
    //cap.set(cv::CAP_PROP_SETTINGS,1);    
    //cap.set(cv::CAP_PROP_FOCUS,38);
    //cap.set(cv::CAP_PROP_AUTO_EXPOSURE,0);
    //cap.set(cv::CAP_PROP_EXPOSURE,-6);


    if(!cap.isOpened())
    {
        printf("无法打开视频");
    }
    Mat frame;
    cap.read(frame);
    LED_POSITION::System ledTrack(frame,1);
    int ff=0;
    namedWindow("src");

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
            circle(frame,pc,15,Scalar(255,255,0),4);
            //rectangle(frame,ledTrack.mTrackBlocks[i].getTrackRect(),Scalar(255,255,0),1);
            int idd=ledTrack.mTrackBlocks[i].getcodeID();
            putText(frame, to_string(idd), pc+Point2f(0,-20), FONT_HERSHEY_COMPLEX, 1.3, Scalar(100, 200, 200), 2);
        }
        stop = getTickCount();
        //printf("Use Time:%ldms\n",(stop-start)/1000000);   
        
        imshow("src",frame);
        int key=waitKey(1);
        if(key=='q') break;
        usleep(9000);
        
        
    }
}