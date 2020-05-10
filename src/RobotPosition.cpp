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

void RobotPosition::Init(Mat img)
{
    //ledtracker init
    vector<int> ids;
    for(auto bot:robots){
        for(auto e:bot.leds){
            ids.push_back(e.first);
        }
    } 
    pLEDtracker->Init(img,ids);
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
    FileStorage cfgfile(cfg,FileStorage::WRITE);

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

void RobotPosition::addrobot(int id,map<int,Eigen::Vector3d> led)
{
    robotInfo in;
    in.id=id;
    in.leds=led;
    in.pose=Eigen::Vector3d(0,0,0);
    robots.push_back(in);
}



bool RobotPosition::position(Mat inputimg)
{
    //led跟踪
    pLEDtracker->run(inputimg);
    map<int,cv::Point2f> leduv = pLEDtracker->getLEDPoint();
    //对每个机器人解析led
    for(int i=0;i<robots.size();i++){
        vector<Eigen::Vector2d> z;
        vector<Eigen::Vector2d> p;
        for(auto led:robots[i].leds){
            auto it=leduv.find(led.first);
            if(it!=leduv.end()){
                Eigen::Vector2d pi = projectuv2xy(it->second,0.39);
                z.push_back(led.second.head(2));
                p.push_back(pi);
                cout<<led.first<<":( "<<pi[0]<<" , "<<pi[1]<<" )"<<endl;
            }
        }
        if(z.size()>=2){
            g2o::SE2 newpose = geomeCalculater.estimpose(z,p,g2o::SE2(robots[i].pose));
            robots[i].pose=newpose.inverse().toVector();
            cout<<"pose:\n"<<robots[i].pose<<endl;
        }
    }
}

Eigen::Vector2d RobotPosition::projectuv2xy(cv::Point2f uv,float h)
{
    Eigen::Matrix3d r=cam_T.rotation().matrix();
    Eigen::Vector3d t=cam_T.translation();
    Eigen::Matrix3d m;
    cv2eigen(cam_M,m);
    Eigen::Matrix3d k;
    k<<r(0,0),r(0,1),r(0,2)*h+t[0],
       r(1,0),r(1,1),r(1,2)*h+t[1],
       r(2,0),r(2,1),r(2,2)*h+t[2];
    Eigen::Vector3d u=(m*k).inverse()*Eigen::Vector3d(uv.x,uv.y,1);
    return Eigen::Vector2d(u[0]/u[2],u[1]/u[2]);
  


   
    
}




}//name