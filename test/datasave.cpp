#include "System.h"
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
#include <fstream>

using namespace cv;
using namespace std;

void v4l2_setting_focus(int val);
void v4l2_setting_fps(int val);

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
        cap.set(cv::CAP_PROP_FPS,25);
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
    LED_POSITION::System ledTrack(frame,1,true,60);
    v4l2_setting_focus(62);//一定要放在cap.read之后。
    v4l2_setting_fps(25);

    namedWindow("src",CV_WINDOW_AUTOSIZE);

    //主循环
    vector<int> ids;
    ids.push_back(5);
    ledTrack.Init(frame,ids);
    //计数
    int cnt=0;
    int savecnt=0;
    ofstream dataout;
    dataout.open("savedata/ledtrackdata.txt");
    if(!dataout.is_open()){
        cout<<"savadata file cant open"<<endl;
    }
    while (cap.read(frame))
    {
        //--------------------主入口
        ledTrack.run(frame,getTickCount());
        
        //----------------画图
        ledTrack.drawObject(frame,LED_POSITION::System::BLOCK);


        Mat resizeimg;
        resize(frame,resizeimg,Size(1280,720));
        imshow("src",resizeimg);
        int key=waitKey(1);
        if(key=='q') break;

        //save
        /*    
        cnt++;
        if(cnt>=5){
            map<int,Point2f> pdata=ledTrack.getLEDPoint();
            if(pdata.size()!=0){
                dataout<<to_string(savecnt)+" "<<to_string(pdata.begin()->second.x)+" "<<
                    to_string(pdata.begin()->second.y)<<endl;
                imwrite("savedata/img/"+to_string(savecnt)+".jpg",frame);
                savecnt++;
            
            }
            cnt=0;
        }
        */
       if(key=='s'){
            map<int,Point2f> pdata=ledTrack.getLEDPoint();
            if(pdata.size()!=0){
                dataout<<to_string(savecnt)+" "<<to_string(pdata.begin()->second.x)+" "<<
                    to_string(pdata.begin()->second.y)<<endl;
                imwrite("savedata/img/"+to_string(savecnt)+".jpg",frame);
                savecnt++;
            }
            else{
                dataout<<to_string(savecnt)+" "<<"null null"<<endl;
                imwrite("savedata/img/"+to_string(savecnt)+".jpg",frame);
                savecnt++;
            }
        }
        else if(key=='q'){
            break;
        }
        
        
    }
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

void v4l2_setting_fps(int  val)
{
     //查看v4l2,通过v4l2设置属性
    int fd = open("/dev/video0", O_RDWR);
    if(fd!=-1){
        struct v4l2_streamparm Stream_Parm;
        memset(&Stream_Parm, 0, sizeof(struct v4l2_streamparm));
        Stream_Parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 

        Stream_Parm.parm.capture.timeperframe.denominator =val;;
        Stream_Parm.parm.capture.timeperframe.numerator = 1;

        int ret = ioctl(fd, VIDIOC_S_PARM, &Stream_Parm);
        if (ret < 0)
                perror("set fps failed (%d)\n"); 
    }
    
}