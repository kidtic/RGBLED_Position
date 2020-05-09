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

using namespace cv;


namespace LED_POSITION
{
#define WORLD_ARTAG 0
#define WORLD_LED 1

//一个RobotPosition代表了一个相机 捕获多个机器人
class RobotPosition
{
public:
    struct robotInfo
    {
        int id;
        //key为其ID。对应了在机器人坐标系的位置
        std::map<int,Eigen::Vector3d> led;
    };
    

private:
    //ledtracker
    LED_POSITION::System* pLEDtracker;

    //config file
    string cfg;
    Mat cam_M;   //相机内参
    Mat cam_diff;   //相机畸变
    g2o::SE3Quat cam_T;      //相机外参


    //artag setworld
    aruco::PREDEFINED_DICTIONARY_NAME ARtag_dict = aruco::DICT_6X6_250;
    uint8_t worldAR_ID=11;         //world artag id
	float worldAR_size= 0.1745;	   //artag real size(m)



public:
    /*
    @brief:需要提供第一帧的图像
    @param:img:提供的第一帧图像
    */
    RobotPosition(Mat img,string cfgpath);
    ~RobotPosition();

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
};



RobotPosition::~RobotPosition()
{
    delete pLEDtracker;
}

}//names

#endif