#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include<time.h>
#include <unistd.h>
#include<set>

using namespace cv;
using namespace std;


//函数
//将idc分配入allclass派系群中
int findinclass(vector<set<int>> allclass,Vec2i idc)
{
    int creatflag=1;
    for(int i=0;i<allclass.size();i++)
    {
        if(allclass[i].count(idc[0]) || allclass[i].count(idc[1]))
        {
            allclass[i].insert(idc[0]);
            allclass[i].insert(idc[1]);
            creatflag=0;
        }
    }
    if(creatflag==1)//说明什么也不属于，新造一个派系
    {
        set<int> inin;
        inin.insert(idc[0]);
        inin.insert(idc[1]);
        allclass.push_back(inin);
    }

    return creatflag;
}

int  main(void)
{
    VideoCapture cap("./res/test_rgb3.mp4");

    if(!cap.isOpened())
    {
        printf("无法打开视频");
    }

    Mat frame,last_frame,last_frame_c;
    Size fchsize(960,540);
    cap.read(frame);
    resize(frame,last_frame,fchsize);
    resize(frame,last_frame_c,fchsize);
    //颜色hsv数据
    Scalar lower_blue(100-5, 60, 90);
    Scalar upper_blue(125+5, 255,255);
    Scalar lower_green(35-5, 60,90);
    Scalar upper_green(77+5, 255,255);
    Scalar lower_red0(156, 60,90);
    Scalar upper_red0(180, 255,255);
    Scalar lower_red1(0, 60,90);
    Scalar upper_red1(10, 255,255);

    namedWindow("src",0);
    vector<Point2f> rg_centers;


    while (cap.read(frame))
    {
        time_t start,stop;
        double totaltime;

         start=getTickCount();
        //降低分辨率
        resize(frame,frame,fchsize);
       
        //转换到hsv
        Mat img_hsv,mask_g,mask_b;
        cvtColor(frame,img_hsv,COLOR_BGR2HSV);
        inRange(img_hsv,lower_green,upper_green,mask_g);
        inRange(img_hsv,lower_blue,upper_blue,mask_b);
        imshow("mask_b",mask_b);
        //last
        Mat last_img_hsv,last_mask_r0,last_mask_r1,last_mask_r;
        cvtColor(last_frame,last_img_hsv,COLOR_BGR2HSV);
        inRange(last_img_hsv,lower_red0,upper_red0,last_mask_r0);
        inRange(last_img_hsv,lower_red1,upper_red1,last_mask_r1);
        bitwise_or(last_mask_r0,last_mask_r1,last_mask_r);

        //对上一帧的图像进行膨胀
        //这里主要目的是要动态差分
        dilate(last_mask_r,last_mask_r,getStructuringElement(MORPH_CROSS,Size(7,7)));
        
        
        //diff
        Mat diff_mask_rg;
        bitwise_and(mask_g,last_mask_r,diff_mask_rg);

        
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        vector<Point2f> listpoint;//用于存储所有轮廓面积大于一定数的轮廓中心点
        findContours(diff_mask_rg, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        if(contours.size()>=2)
        {
            cout<<"轮廓数为"<<contours.size()<<endl;
            for (int i = 0; i < contours.size(); i++) 
            {	
                //Scalar color = Scalar(255,0,0);
                //drawContours(frame, contours, i, color, 4, 8, hierarchy, 0, Point(0, 0));
                //计算轮廓的矩
                Moments M = moments(contours[i]);
                //计算轮廓中心点
                int cx = int(M.m10/M.m00);
                int cy = int(M.m01/M.m00);
                //计算轮廓面积
                double Sarea=contourArea(contours[i]);
                //cout<<"  cntID:"<<i<<" 中心点："<<cx<<","<<cy<<" 面积："<<Sarea<<endl;
                //把满足要求的中心点push进listpoint
                if(Sarea>5)
                {
                    listpoint.push_back(Point2f(cx,cy));
                }
            }
            


            //----------------------------------------对listpoint聚类一次，消除邻近点
            vector<int> classidcount; //记录CLASSID号的个数
            vector<int> classid;
            for (size_t i = 0; i < listpoint.size(); i++)
            {
                classidcount.push_back(1);
                classid.push_back(i);
            }
            //组合数，计算两两之间的距离
            vector<Vec2i> listpoint_connect;//需要聚类的点
            double clusteresp=10;  //领域距离
            for(int i=0;i<listpoint.size();i++)
            {
                for(int j=i+1;j<listpoint.size();j++)
                {
                    Point2f vv=listpoint[i]-listpoint[j];
                    double juli= sqrt(vv.x*vv.x+vv.y*vv.y);
                    if(juli<clusteresp)
                    {
                        listpoint_connect.push_back(Vec2i(i,j));
                    }
                }
            }

            //通过感染的方式，进行聚类
            for(int i =0;i<listpoint_connect.size();i++)
            {
                //提出这两个点的classid
                int p0id=classid[listpoint_connect[i][0]];
                int p1id=classid[listpoint_connect[i][1]];
                //首先判断这两个点所属的群数量谁大
                if(classidcount[p0id]>=classidcount[p1id])
                {
                    //p1id数量小，所以p0id感染所有p1id的点
                    for(int j=0;j<classid.size();j++)
                    {
                        if(classid[j]==p1id) classid[j]=p0id;
                    }
                    classidcount[p0id]+=classidcount[p1id];
                    classidcount[p1id]=0;
                }
                else
                {
                    //p0id数量小，所以p1id感染所有p0id的点
                    for(int j=0;classid.size();j++)
                    {
                        if(classid[j]==p0id) classid[j]=p1id;
                    }
                    classidcount[p1id]+=classidcount[p0id];
                    classidcount[p0id]=0;
                }
            }    


            //--------------------------------------计算中心点
            //计算classNum
            int classNum=0;
            for(int i=0;i<classidcount.size();i++)
            {
                if(classidcount[i]!=0)classNum++;
            }

            vector<Point2f> rg_centers_c(classNum,Point2f(0,0));
            vector<int> rg_centers_c_count(classNum,0);
            
            for(int i=0;i<listpoint.size();i++)
            {
                int clss=classid[i];
                if(clss>=0)
                {
                    rg_centers_c_count[clss]++;
                    rg_centers_c[clss]+=listpoint[i];
                }
            }
            for(uint j=0;j<classNum;j++)
            {
                rg_centers_c[j]=rg_centers_c[j]/rg_centers_c_count[j];
            }
            rg_centers=rg_centers_c;

        }
        stop = getTickCount();
        printf("Use Time:%ldms\n",(stop-start)/1000000);        
        

    
       
        //画圈
        
        for(int i=0;i<rg_centers.size();i++)
        {
            circle(frame,rg_centers[i],15,Scalar(255,255,0),3);
        }
        

        imshow("src",frame);
        
        waitKey(0);

        last_frame=last_frame_c.clone();
        last_frame_c=frame.clone();
    }
    
    return 0;
}


