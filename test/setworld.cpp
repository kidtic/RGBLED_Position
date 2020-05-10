#include "RobotPosition.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>



void v4l2_setting_focus(int val);


int main(int argc, char const *argv[])
{
    VideoCapture cap;
    cout<<argc<<argv[0]<<endl;
    if(argc==2 && string(argv[1])=="rtsp"){
        string rtspstr="rtsp://admin:zou133zzq@192.168.123.110:554/cam/realmonitor?channel=1&amp;subtype=0&amp;unicast=true&amp;proto=Onvif";
        cap.open(rtspstr);
    }
    else if(argc==2 && string(argv[1])=="red"){
        cap.open("/home/kk/dataset/led/aimibot.mp4");
    }
    else if(argc==2 && string(argv[1])=="0"){
        cap.open(0);
        //相机参数调整
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
        cap.set(cv::CAP_PROP_FRAME_WIDTH,1920);
        //usleep(1000);
        cap.set(cv::CAP_PROP_FPS,30);
        //cap.set(cv::CAP_PROP_FOCUS,10);
    }
    else{
        printf("need input arg:\n  rtsp  red  0\n");
        return -1;
    }
    //判断视频是否打开
    if(cap.isOpened()==false){
        perror("video can't open\n");
        return -1;
    }
    
    Mat frame;
    cap.read(frame);
    LED_POSITION::RobotPosition robot(frame,"config/camPosi_cfg.json");
    v4l2_setting_focus(50);//一定要放在cap.read之后。
    
    while(cap.read(frame))
    {
        robot.drawWorldtoShow(frame);
        int key=waitKey(10);
        if(key=='s'){
            robot.setWorld(frame);
        }
        else if(key=='q'){
            break;
        }
    }

    return 0;

}


void v4l2_setting_focus(int val)
{
    //查看v4l2,通过v4l2设置属性
    int fd = open("/dev/video0", O_RDWR);
    if(fd!=-1){
        struct v4l2_control ctrl;
        ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
        if(ioctl(fd, VIDIOC_G_CTRL, &ctrl)!=-1){
            cout<<"V4L2_CID_FOCUS_ABSOLUTE="<<ctrl.value<<endl;
            cout<<"set V4L2_CID_FOCUS_ABSOLUTE......"<<endl;
            struct v4l2_control ctrl1;
            ctrl1.id=V4L2_CID_FOCUS_AUTO;
            ctrl1.value=0;
            int ret=ioctl(fd,VIDIOC_S_CTRL,&ctrl1);
            if (ret < 0)
                perror("unset focus_auto  failed (%d)\n"); 
            ctrl1.id=V4L2_CID_FOCUS_ABSOLUTE;
            ctrl1.value=val;
            ret=ioctl(fd,VIDIOC_S_CTRL,&ctrl1);
            if (ret < 0)
                perror("set focus failed (%d)\n"); 
        }

    }
    else
        perror("can't open /dev/video0");
    close(fd);

}