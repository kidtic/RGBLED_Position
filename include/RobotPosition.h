#ifndef ROBOTPOSITION_H_
#define ROBOTPOSITION_H_

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "eigen3/Eigen/Core"
#include "System.h"
#include <map>

#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "GeomeOperation.h"

using namespace cv;


namespace LED_POSITION
{
#define WORLD_ARTAG 0
#define WORLD_LED 1
class geomeopera;
//一个RobotPosition代表了一个相机 捕获多个机器人
class RobotPosition
{
public:
    struct robotInfo
    {
        int id;
        //key为其ID。对应了在机器人坐标系的位置
        std::map<int,Eigen::Vector3d> leds;
        //position 后坐标(x,y,theta)
        Eigen::Vector3d pose;
    };
    
    //ledtracker
    LED_POSITION::System* pLEDtracker;
private:
    
    //cu
    LED_POSITION::geomeopera geomeCalculater;

    //config file
    string cfg;
    Mat cam_M;   //相机内参
    Mat cam_diff;   //相机畸变
    g2o::SE3Quat cam_T;      //相机外参


    //artag setworld
    aruco::PREDEFINED_DICTIONARY_NAME ARtag_dict = aruco::DICT_6X6_250;
    uint8_t worldAR_ID=11;         //world artag id
	float worldAR_size= 0.1745;	   //artag real size(m)

    //机器人数量以及其信息
    vector<robotInfo> robots;



public:
    /*
    @brief:需要提供第一帧的图像
    @param:img:提供的第一帧图像
    */
    RobotPosition(Mat img,string cfgpath);
    ~RobotPosition();

    void Init(Mat img);

    /*
    * @brief：识别世界坐标系并且imshow出来，但是不会set
    * @param：flag:是采用ARtag 还是采用LED机器人
    *         inputimg:输入的图片
    */
    void drawWorldtoShow(Mat inputimg, int flag=WORLD_ARTAG);

    /*
    * @brief：设立世界坐标系，输入一个带有ARtab或者有机器人的图像,然后就
    *        可以生成cam_T相机外参
    * @param：flag:是采用ARtag 还是采用LED机器人
    *         inputimg:输入的图片
    */
    bool setWorld(Mat inputimg, int flag=WORLD_ARTAG);
    /*
    * @brief：保存配置文件
    */
    void saveCameraConfig();
    void loadCameraConfig();

    /*
    * @brief：添加机器人信息
    * @param：id:机器人的ID号
    *         led：led定位器的编号与坐标
    */
   void addrobot(int id,map<int,Eigen::Vector3d> led);

    /*
    * @brief：定位，提供一张照片，返回机器人位姿(二维),由于采用led跟踪，
    *       所以position需要按照帧率严格的连续调用。
    * @param：inputimg：输入图片
    *         
    */
    bool position(Mat inputimg);

    /*
    * @brief：对于定高的机器人来说，只需要给其led的像素坐标即可求出空间xy坐标
    * @param：uv 像素坐标
    *         h，对应的led点的高
    */
    Eigen::Vector2d projectuv2xy(cv::Point2f uv,float h);

    /*
    * @brief：获取id号的机器人的位姿 
    * @param：id robot id
    *         p: robot's position
    *         q: robot's quater
    */
    void getRobotPose(int id,Eigen::Vector3d &p, Eigen::Quaterniond &q);
    void getRobotPose(int id,Eigen::Vector3d &p, Eigen::Vector3d &rpy);



};



RobotPosition::~RobotPosition()
{
    delete pLEDtracker;
}

}//names

#endif