#include"ScanRelocate.h"


namespace LED_POSITION
{

ScanRelocate::ScanRelocate(System *pSys,Mat frame0)
{
    mpSystem=pSys;
    mTrackBlockCodeLen=mpSystem->getTrackBlockCodeLen();
    mTrackBlockWidth_Sarea=mpSystem->getTrackBlockWidth_Sarea();
    lowSize=mpSystem->getLowSize();
    
    frame=frame0.clone();
    frame_last=frame.clone();
    frame_pre=frame.clone();
    
    sizek=frame.cols/lowSize.width;
    cout<<"lowsize:"<<lowSize<<endl;
    cout<<"mTrackBlockCodeLen:"<<mTrackBlockCodeLen<<endl;
    cout<<"mTrackBlockWidth_Sarea:"<<mTrackBlockWidth_Sarea<<endl;

    resize(frame,re_frame,lowSize);
    
    re_frame_last=re_frame.clone();
    re_frame_pre=re_frame.clone();

    lower_blue=Scalar(100-5, 60, 140);
    upper_blue=Scalar(125+5, 255,255);
    lower_green=Scalar(35-5, 60,140);
    upper_green=Scalar(77+5, 255,255);
    
    lower_red0=Scalar(156, 60,120);
    upper_red0=Scalar(180, 255,255);
    lower_red1=Scalar(0, 60,120);
    upper_red1=Scalar(10, 255,255);
    mScanCntFlag=0;
    
}



ScanRelocate::~ScanRelocate()
{
}


int ScanRelocate::scanFrame()
{
    int ret=0;

    //如果检测到有轮廓点，那么就再检测一次下一帧的轮廓，
    //两次轮廓点的聚类结果就是要创建的跟踪块
    
    //差分（针对低分辨率）
    Mat diff_mask_rg = diffFrame('G','R');
    //imshow("diff",diff_mask_rg);
    //waitKey(1);
    vector<double> Sarea;
    vector<Point2f> contourCenters = mpSystem->findContourCenter(diff_mask_rg,Sarea);
    
    if( mScanCntFlag==0 &&contourCenters.size()>0)
    {
        mScanCntFlag=1;
        scanContourCenters_c=contourCenters;
        scanSarea_c=Sarea;
        ret=0;
    }
    else if(mScanCntFlag==1)
    {
        mScanCntFlag=0;
        //两次结果聚类contourCenters与initContourCenters_c
        contourCenters.insert(contourCenters.end(),scanContourCenters_c.begin(),scanContourCenters_c.end());
        Sarea.insert(Sarea.end(),scanSarea_c.begin(),scanSarea_c.end());
        //计算平均面积
        double Sarea_avg=0;
        for (size_t i = 0; i < Sarea.size(); i++)
        {
            Sarea_avg+=Sarea[i];
        }
        Sarea_avg=Sarea_avg/Sarea.size();
        //设计跟踪块大小
        int mTrackBlockWidth=mTrackBlockWidth_Sarea*sqrt(Sarea_avg)*sizek;
        //聚类
        mpSystem->mCluster.initdata(contourCenters,2*sqrt(Sarea_avg));
        mpSystem->mCluster.clustering();
        vector<Point2f> ledct;
        mpSystem->mCluster.getCenter(ledct);

        //创建跟踪
        if(Sarea_avg>3)
        {
            for (size_t i = 0; i < ledct.size(); i++)
            {
                //cout<<ledct[i]*sizek<<"     ,      "<<mTrackBlockWidth<<endl;
                Point2f iniCt=ledct[i]*sizek;
                //判断是否超界
                if((iniCt.x+mTrackBlockWidth/2)<frame.cols-1&&
                    (iniCt.x-mTrackBlockWidth/2)>1 &&
                    (iniCt.y+mTrackBlockWidth/2)<frame.rows-1&&
                    (iniCt.y-mTrackBlockWidth/2)>1  )
                    {
                        //cout<<ledct[i]*sizek<<"     ,      "<<mTrackBlockWidth<<endl;
                        mTrackBlocks.push_back(TrackBlock(&mMutexNO,frame,ledct[i]*sizek,mTrackBlockWidth,mTrackBlockCodeLen));
                        ret=1;
                    }
            }
        }
       
    }
    

    return ret;
}

void ScanRelocate::Run()
{
    int status=0;

    while (1)
    {
        time_t start,stop;
        double totaltime;
        if(mpSystem->getInputScanRelocateFlag())
        {
            start=getTickCount();
            mpSystem->setInputScanRelocateFlag(false);
            updataFromSys();
            //检查是否有losting的tb
            if (checkTBStatusLOSTING())
            {
                
                if(status==0)
                {
                    status=scanFrame();
                }
                else if(status==1)
                {
                    for (size_t i = 0; i < mTrackBlocks.size(); i++)
                    {
                        mTrackBlocks[i].track(frame);
                    }
                    //恢复sysBT
                    recoverBT();
                    //更具ID识别结果来删除
                    delateTrackBlock(-1);

                    if(mTrackBlocks.size()==0) status=0;
                }

                
            }
            else
                status=0;


            //历史数据
            //frame_pre=frame_last.clone();
            //frame_last=frame.clone();
            re_frame_pre=re_frame_last.clone();
            re_frame_last=re_frame.clone();

            stop = getTickCount();
            //printf("ScanRelocate Use Time:%ldms\n",(stop-start)/1000000);   
        }
        else
        {
            usleep(1000);
        }

    }
}

void ScanRelocate::updataFromSys()
{
   
    vector<Point2f> cTBCenter;
    vector<Rect> cTBRect;
    vector<int> cTBID;
    vector<int> cTBStatus;
    for (size_t i = 0; i < mpSystem->mTrackBlocks.size(); i++)
    {
        cTBCenter.push_back(mpSystem->mTrackBlocks[i].getCenter());
        cTBID.push_back(mpSystem->mTrackBlocks[i].getcodeID());
        cTBStatus.push_back(mpSystem->mTrackBlocks[i].getStatus());
        cTBRect.push_back(mpSystem->mTrackBlocks[i].getTrackRect());
    }
    mTBCenter=cTBCenter;
    mTBID=cTBID;
    mTBStatus=cTBStatus;
    mTBRect=cTBRect;

    frame=mpSystem->getFrame(0).clone();

    for(int i=0;i<cTBRect.size();i++){
        if(cTBStatus[i]!=1){
            Mat imag = cv::Mat::zeros(cTBRect[i].height,cTBRect[i].width, CV_8UC3);
            imag.copyTo(frame(cTBRect[i]));
        }
        
    }
    resize(frame,re_frame,lowSize);
    //imshow("scan",frame);
    
}

bool ScanRelocate::checkTBStatusLOSTING()
{
    for (size_t i = 0; i < mTBStatus.size(); i++)
    {
        /* code */
        if(mTBStatus[i]==1) return true;
    }
    return false;
    
}

Mat ScanRelocate::diffFrame(char nowColor,char lastColor)
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
    
    //diffS
    Mat diff_mask_rg;
    bitwise_and(mask_g,last_mask_r,diff_mask_rg);

    return diff_mask_rg;
}
//-1:把没要的TrackBlock 删掉（状态不是准备态的）
void ScanRelocate::delateTrackBlock(int index)
{
    if(index==-1)
    {
        vector<TrackBlock>::iterator it;
        for (it=mTrackBlocks.begin();it!=mTrackBlocks.end();)
        {
            if(it->getStatus()!=TrackBlock::PREPARE)
            {
                printf("dalate TB\n");
                it=mTrackBlocks.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
    else if(index>=0)
    {
        mTrackBlocks.erase(mTrackBlocks.begin()+index);
    }
    
}

void ScanRelocate::recoverBT()
{
    for (size_t i = 0; i < mTBStatus.size(); i++)
    {
        if(mTBStatus[i]==1)
        {
            
            int id=mTBID[i];
            //cout<<"已经丢失的 ID："<<id<<endl;
            //看看在本类tb中，能否找到一样的id
            for(int j=0;j<mTrackBlocks.size();j++)
            {
                if(id==mTrackBlocks[j].getcodeID())
                {
                    cout<<"恢复ID："<<id<<endl;
                    mpSystem->mTrackBlocks[i].setStatus(TrackBlock::OK);
                    mpSystem->mTrackBlocks[i].setCenter(mTrackBlocks[j].getCenter());
                    mpSystem->mTrackBlocks[i].setTrackRect(mTrackBlocks[j].getTrackRect());


                }
            }
        }
    }
    
}



}

