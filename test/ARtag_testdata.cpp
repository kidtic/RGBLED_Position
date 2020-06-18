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
#include <fstream>

using namespace cv;
using namespace std;

void v4l2_setting_focus(int val);
map<int,Point2f> artag_detecte(Mat& input);

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
    
    Mat srcframe;
    cap.read(srcframe);
    v4l2_setting_focus(50);//一定要放在cap.read之后。

    namedWindow("src",CV_WINDOW_AUTOSIZE);

    //主循环

    //计数
    int cnt=0;
    int savecnt=0;
    ofstream dataout;
    dataout.open("savedata/ARTAGdata.txt");
    if(!dataout.is_open()){
        cout<<"savadata file cant open"<<endl;
    }
    while (cap.read(srcframe))
    {
        Mat frame=srcframe.clone();
       map<int,Point2f> pdata=artag_detecte(frame);

        Mat resizeimg;
        resize(frame,resizeimg,Size(1280,720));
        imshow("src",resizeimg);
        int key=waitKey(1);
        if(key=='q') break;

        //save    
        if(key=='s'){
            if(pdata.size()!=0){
                dataout<<to_string(savecnt)+" "<<to_string(pdata.begin()->second.x)+" "<<
                    to_string(pdata.begin()->second.y)<<endl;
                imwrite("savedata/img/"+to_string(savecnt)+".jpg",srcframe);
                savecnt++;
            }
            else{
                dataout<<to_string(savecnt)+" "<<"null null"<<endl;
                imwrite("savedata/img/"+to_string(savecnt)+".jpg",srcframe);
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

map<int,Point2f> artag_detecte(Mat& input){
    map<int,Point2f> res;
    //相机坐标系的位姿信息
    vector< Vec3d > rvecs, tvecs;
    vector<int> ids;
    //dict
    Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    //开始计算
    vector<vector<Point2f> > corners;
    cv::aruco::detectMarkers(input, dictionary, corners, ids);
    // 如果有AR码，则进行姿态估计
    if (ids.size() > 0)
    {
        for(int i=0;i<ids.size();i++){
            vector<Point2f> corner=corners[i];
            //计算中心点
            float dy13 = corner[0].y - corner[2].y;
            float dx13 = corner[0].x - corner[2].x;
            float dy24 = corner[1].y - corner[3].y;
            float dx24 = corner[1].x - corner[3].x;
            
            Eigen::Matrix2f K;
            K << dy13, -dx13,
                dy24, -dx24;
            Eigen::Vector2f B;
            B << dy13*corner[0].x - dx13*corner[0].y, 
                dy24*corner[1].x - dx24*corner[1].y;
            Eigen::Vector2f retxy;
            retxy = K.inverse()*B;
            Point2f ret;
            ret.x = retxy[0]; ret.y = retxy[1];
            res[ids[i]]=ret;
            aruco::drawDetectedMarkers(input, corners, ids, Scalar(0, 255, 0));
        } 
    }

    return res;
    
}