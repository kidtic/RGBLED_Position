#include <RobotPosition.h>
#include <opencv2/core/eigen.hpp>

namespace LED_POSITION
{

RobotPosition::RobotPosition(Mat img,string cfgpath)
{
    pLEDtracker=new System(img,1,true);
    cfg=cfgpath;
    
    //load cfg
    loadCameraConfig();
    cout<<"strat:"<<cam_T<<endl;


}


void RobotPosition::drawWorldtoShow(Mat inputimg, int flag)
{
    Mat drawimg;
    inputimg.copyTo(drawimg);
    //相机坐标系的位姿信息
    vector< Vec3d > rvecs, tvecs;
    vector<int> ids;
    //dict
    Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(ARtag_dict);

    //开始计算
    vector<vector<Point2f> > corners;
    cv::aruco::detectMarkers(inputimg, dictionary, corners, ids);
    // 如果有AR码，则进行姿态估计
    if (ids.size() > 0)
    {
        cv::aruco::estimatePoseSingleMarkers(corners, worldAR_size, cam_M, cam_diff, rvecs, tvecs);
        cv::aruco::drawAxis(drawimg, cam_M, cam_diff, rvecs[0], tvecs[0], 1); 
    }
    
    
    imshow("drawAxis",drawimg);


}

bool RobotPosition::setWorld(Mat inputimg, int flag)
{
    
    //相机坐标系的位姿信息
    vector< Vec3d > rvecs, tvecs;
    vector<int> ids;
    //dict
    Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(ARtag_dict);

    //开始计算
    vector<vector<Point2f> > corners;
    cv::aruco::detectMarkers(inputimg, dictionary, corners, ids);
    // 如果有AR码，则进行姿态估计
    if (ids.size() > 0)
    {
        cv::aruco::estimatePoseSingleMarkers(corners, worldAR_size, cam_M, cam_diff, rvecs, tvecs);
        //得到（世界坐标系的RT）
        Mat rmat;
      
        cv::Rodrigues(rvecs[0], rmat);
        Eigen::Matrix3d r;
        cv2eigen(rmat,r);
        Eigen::Vector3d t;
        cv2eigen(tvecs[0],t);
        cam_T=g2o::SE3Quat(r,t);
        cout<<"cam_T:\n"<<cam_T<<endl;
        saveCameraConfig();
        return true;
    }
    else
        return false;

}
    
    
void RobotPosition::saveCameraConfig()
{
    FileStorage cfgfile(cfg,FileStorage::APPEND);

    Mat r,t;
    eigen2cv(cam_T.rotation().matrix(),r);
    eigen2cv(cam_T.translation(),t);
    cfgfile.write("cam_mtx",cam_M);
    cfgfile.write("cam_diff",cam_diff);
    cfgfile.write("cam_r",r);
    cfgfile.write("cam_t",t);
    
    cfgfile.release();
}

void RobotPosition::loadCameraConfig()
{
    FileStorage cfgfile(cfg,FileStorage::READ);
    if(!cfgfile.isOpened()){
        perror("can't open config file");
    }
    Mat r,t;
    cfgfile["cam_r"]>>r;
    cfgfile["cam_t"]>>t;
    Eigen::Matrix3d re;
    Eigen::Vector3d te;
    cv2eigen(r,re);
    cv2eigen(t,te);
    cam_T=g2o::SE3Quat(re,te);

    cfgfile["cam_mtx"]>>cam_M;
    cfgfile["cam_diff"]>>cam_diff;

    cfgfile.release();
}




}//name