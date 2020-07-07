#include <RobotPosition.h>
#include <opencv2/core/eigen.hpp>
#include <algorithm>

namespace LED_POSITION
{

RobotPosition::RobotPosition(Mat img,string cfgpath)
{
    pLEDtracker=new System(img,1,true,80);
    cfg=cfgpath;
    
    //load cfg
    loadCameraConfig();
    cout<<"strat:"<<cam_T<<endl;
    
    loadJsonConfig();
    cout<<"MarkerPoint.size   "<<MarkerPoint.size()<<endl;



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
    
    if(flag==WORLD_ARTAG){
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
    }
    else if(flag==2){
        Vec3d rvec,tvec;
        

        eigen2cv(cam_T.translation(),tvec);

        Eigen::Quaterniond qi=cam_T.rotation();
        Mat casheid;
        eigen2cv(qi.matrix(),casheid);
        cv::Rodrigues(casheid, rvec);
        cv::aruco::drawAxis(drawimg, cam_M, cam_diff, rvec, tvec, 1);
    }
    
    
    
    imshow("drawAxis",drawimg);


}

void RobotPosition::drawWorldtoShow(Mat inputimg,vector<Point2f> imagePoint)
{
    Mat drawimg;
    inputimg.copyTo(drawimg); 

    if(MarkerPoint.size()==imagePoint.size() && imagePoint.size()>=4){
        Vec3d rvec, tvec;
        bool ret=cv::solvePnP(MarkerPoint,imagePoint,cam_M,cam_diff,rvec,tvec);
        if(ret==true){
            cv::aruco::drawAxis(drawimg, cam_M, cam_diff, rvec, tvec, 1);
        }
    }
    
    
    //imagePoint 十字
    for(int i=0;i<imagePoint.size();i++){
        cv::circle(drawimg,imagePoint[i],1,Scalar(255,0,0));
    }

    //显示
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

bool RobotPosition::setWorld(vector<Point2f> imagePoint){
    Vec3d rvec, tvec;
    bool ret=cv::solvePnP(MarkerPoint,imagePoint,cam_M,cam_diff,rvec,tvec);
    if(ret==true){
        Mat rmat;
      
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d r;
        cv2eigen(rmat,r);
        Eigen::Vector3d t;
        cv2eigen(tvec,t);
        cam_T=g2o::SE3Quat(r,t);
        cout<<"cam_T:\n"<<cam_T<<endl;
        saveCameraConfig();
    }
    return ret;
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

    //json
    //json
    Json::Value json_root;
    Json::Reader reader;
    ifstream fs(cfg,ios::binary);
    if(!reader.parse(fs,json_root)){
        cout << "json data open error" << endl;
		fs.close();
    }
    else{
        cout<<json_root.toStyledString()<<endl;
    }
    for(int i=0;i<MarkerPoint.size();i++){
        Json::Value pp;
        pp.append(MarkerPoint[i].x);
        pp.append(MarkerPoint[i].y);
        pp.append(MarkerPoint[i].z);
        json_root["MarkerPoint"].append(pp);
    }

    Json::StyledWriter wt;
    string str = wt.write(json_root);
    ofstream ofs(cfg);
	ofs << str;
	ofs.close();



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



bool RobotPosition::position(Mat inputimg,int64 timesp,bool prtpose)
{
    //led跟踪
    pLEDtracker->run(inputimg,timesp);
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
                //cout<<led.first<<":( "<<pi[0]<<" , "<<pi[1]<<" )"<<endl;
            }
        }
        if(z.size()>=2){
            g2o::SE2 pose0(0,0,0);
            g2o::SE2 newpose = geomeCalculater.estimpose(z,p,pose0);
            robots[i].pose=newpose.inverse().toVector();
            if(prtpose)cout<<"pose:\n"<<robots[i].pose<<endl;
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

void RobotPosition::getRobotPose(int id,Eigen::Vector3d &p, Eigen::Quaterniond &q)
{
    for(int i=0;i<robots.size();i++){
        if(robots[i].id==id){
            p=Eigen::Vector3d(robots[i].pose[0],robots[i].pose[1],0);
            Eigen::Quaterniond drpyQ;//两帧的旋转欧拉角
            drpyQ = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * 
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                    Eigen::AngleAxisd(robots[i].pose[2], Eigen::Vector3d::UnitZ());
            q=drpyQ;
  
        }
    }
    
}
void RobotPosition::getRobotPose(int id,Eigen::Vector3d &p, Eigen::Vector3d &rpy)
{
    for(int i=0;i<robots.size();i++){
        if(robots[i].id==id){
            p=Eigen::Vector3d(robots[i].pose[0],robots[i].pose[1],0);
            rpy=Eigen::Vector3d(0,0,robots[i].pose[2]);
        
        }
    }
}

void RobotPosition::loadJsonConfig()
{
    //json
    Json::Value json_root;
    Json::Reader reader;
    ifstream fs(cfg,ios::binary);
    if(!reader.parse(fs,json_root)){
        cout << "json data open error" << endl;
		fs.close();
    }

    //MarkerPoint
    for(int i=0;i<json_root["MarkerPoint"].size();i++){
        if(json_root["MarkerPoint"][i].size()!=3){
            perror("cfg MarkerPoint 格式有误，应该为3列");
            return;
        }
        float x=json_root["MarkerPoint"][i][0].asFloat();
        float y=json_root["MarkerPoint"][i][1].asFloat();
        float z=json_root["MarkerPoint"][i][2].asFloat();
        MarkerPoint.push_back(Point3f(x,y,z));
  
    }
}






}//name