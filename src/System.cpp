#include"System.h"

namespace LED_POSITION
{
    
System::System(Mat frame0,double resizek)
{
    mpScanRelocater = new ScanRelocate(this);

    lower_blue=Scalar(100-5, 60, 140);
     upper_blue=Scalar(125+5, 255,255);
     lower_green=Scalar(35-5, 60,140);
     upper_green=Scalar(77+5, 255,255);
     
     lower_red0=Scalar(156, 60,120);
     upper_red0=Scalar(180, 255,255);
     lower_red1=Scalar(0, 60,120);
     upper_red1=Scalar(10, 255,255);

     frame=frame0.clone();
     frame_last=frame0.clone();
     frame_pre=frame0.clone();

     sizek=resizek;
     lowSize=Size(frame.cols/sizek,frame.rows/sizek);

     resize(frame,re_frame,lowSize);
     re_frame_last=re_frame.clone();
     re_frame_pre=re_frame.clone();

     initCntFlag=0;


     //初始化重定位线程
     mpScanRelocater=new ScanRelocate(this);
     mptScanRelocate = new thread(&LED_POSITION::ScanRelocate::Run, mpScanRelocater);
     
     

}

System::~System()
{

}

int System::Init(Mat frameInput)
{
    int ret;
    frame=frameInput;
    resize(frame,re_frame,lowSize);

    //如果检测到有轮廓点，那么就再检测一次下一帧的轮廓，
    //两次轮廓点的聚类结果就是要创建的跟踪块
    
    //差分（针对低分辨率）
    Mat diff_mask_rg = diffFrame('G','R');
    //imshow("diff",diff_mask_rg);
    vector<double> Sarea;
    vector<Point2f> contourCenters = findContourCenter(diff_mask_rg,Sarea);
    
    if( initCntFlag==0 &&contourCenters.size()>0)
    {
        initCntFlag=1;
        initContourCenters_c=contourCenters;
        initSarea_c=Sarea;
        ret=0;
    }
    else if(initCntFlag==1)
    {
        initCntFlag=0;
        //两次结果聚类contourCenters与initContourCenters_c
        contourCenters.insert(contourCenters.end(),initContourCenters_c.begin(),initContourCenters_c.end());
        Sarea.insert(Sarea.end(),initSarea_c.begin(),initSarea_c.end());
        //计算平均面积
        double Sarea_avg=0;
        for (size_t i = 0; i < Sarea.size(); i++)
        {
            Sarea_avg+=Sarea[i];
        }
        Sarea_avg=Sarea_avg/Sarea.size();
        //设计跟踪块大小
        mTrackBlockWidth=mTrackBlockWidth_Sarea*sqrt(Sarea_avg)*sizek;
        //聚类
        mCluster.initdata(contourCenters,2*sqrt(Sarea_avg));
        mCluster.clustering();
        vector<Point2f> ledct;
        mCluster.getCenter(ledct);
        //创建跟踪块
        for (size_t i = 0; i < ledct.size(); i++)
        {
            mTrackBlocks.push_back(TrackBlock(frame,ledct[i]*sizek,mTrackBlockWidth,mTrackBlockCodeLen));
        }
        
        

        ret=1;
    }
    


    frame_pre=frame_last.clone();
    frame_last=frame.clone();
    re_frame_pre=re_frame_last.clone();
    re_frame_last=re_frame.clone();
    return ret;
}

void System::position(Mat frameInput)
{
    frame=frameInput.clone();
    resize(frame,re_frame,lowSize);


    
    for (size_t i = 0; i < mTrackBlocks.size(); i++)
    {
        mTrackBlocks[i].track(frame);
    }
    
    frame_pre=frame_last.clone();
    frame_last=frame.clone();
    re_frame_pre=re_frame_last.clone();
    re_frame_last=re_frame.clone();
}



Mat System::diffFrame(char nowColor,char lastColor)
{
    
    //转换到hsv
    Mat img_hsv,mask_g;
    cvtColor(re_frame,img_hsv,COLOR_BGR2HSV);
    inRange(img_hsv,lower_green,upper_green,mask_g);
    //last
    Mat last_img_hsv,last_mask_r0,last_mask_r1,last_mask_r;
    cvtColor(re_frame_pre,last_img_hsv,COLOR_BGR2HSV);
    inRange(last_img_hsv,lower_red0,upper_red0,last_mask_r0);
    inRange(last_img_hsv,lower_red1,upper_red1,last_mask_r1);
    bitwise_or(last_mask_r0,last_mask_r1,last_mask_r);

    //对上一帧的图像进行膨胀
    //这里主要目的是要动态差分
    dilate(last_mask_r,last_mask_r,getStructuringElement(MORPH_CROSS,Size(7,7)));
    
    //diff
    Mat diff_mask_rg;
    bitwise_and(mask_g,last_mask_r,diff_mask_rg);

    return diff_mask_rg;
}

vector<Point2f> System::findContourCenter(Mat inputImg ,vector<double> &Sarea)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point2f> retpoint;//用于存储所有轮廓面积大于一定数的轮廓中心点
    findContours(inputImg, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); i++) 
    {	
        //计算轮廓的矩
        Moments M = moments(contours[i]);
        //计算轮廓中心点
        int cx = int(M.m10/M.m00);
        int cy = int(M.m01/M.m00);
        //计算轮廓面积
        double S=contourArea(contours[i]);
        //把满足要求的中心点push进listpoint
        if(S>0)
        {
            retpoint.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    return retpoint;


}

}//namespace LED_POSITION
