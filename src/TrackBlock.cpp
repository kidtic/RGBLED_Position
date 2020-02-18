#include"TrackBlock.h"


namespace LED_POSITION
{

TrackBlock::TrackBlock(std::mutex *pMutex,Mat srcinput,Point2f initPoint,int rectwidth,int mcodeLength)
{
    //mpSystem=pSys;
    pSysMutex=pMutex;
    //setCenter(initPoint);
    ledCenter=initPoint;
    codeLength=mcodeLength;
    mVelocity=Point2f(0,0);
    //mContourArea=sarea;
    //mMutexCenter.lock();
    rect=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    rect_last=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    rect_pre=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    //mMutexCenter.unlock();
    
    //刚开始处于准备状态
    //setStatus(PREPARE);
    TrackStatus=PREPARE;

    //codeCashe
    for (size_t i = 0; i < codeLength; i++)
    {
        codeCashe.push_back(-1);
    }
    //setcodeID(-1);
    codeID=-1;
    mpReadCodeCNT=0;
    mpReadCodeImg=mpReadCodeImg_fp;

    //初始化图像块
    srcImgSize=srcinput.size();
    BlockImg=srcinput(rect).clone();
    BlockImg_last=BlockImg.clone();
    BlockImg_pre=BlockImg.clone();

    //颜色常量
    lower_blue=Scalar(100-5, 60, 140);
    upper_blue=Scalar(125+5, 255,255);
    lower_green=Scalar(35-5, 60,140);
    upper_green=Scalar(77+5, 255,255);

    lower_red0=Scalar(156, 60,120);
    upper_red0=Scalar(180, 255,255);
    lower_red1=Scalar(0, 60,120);
    upper_red1=Scalar(10, 255,255);
    
}
TrackBlock::TrackBlock(Mat srcinput,Point2f initPoint,int rectwidth,int mcodeLength)
{

    //setCenter(initPoint);
    ledCenter=initPoint;
    codeLength=mcodeLength;
    mVelocity=Point2f(0,0);
    //mContourArea=sarea;
    //mMutexCenter.lock();
    rect=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    rect_last=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    rect_pre=Rect(ledCenter.x-(rectwidth/2),ledCenter.y-(rectwidth/2),rectwidth,rectwidth);
    //mMutexCenter.unlock();
    
    //刚开始处于准备状态
    //setStatus(PREPARE);
    TrackStatus=PREPARE;

    //codeCashe
    for (size_t i = 0; i < codeLength; i++)
    {
        codeCashe.push_back(-1);
    }
    //setcodeID(-1);
    codeID=-1;
    mpReadCodeCNT=0;
    mpReadCodeImg=mpReadCodeImg_fp;

    //初始化图像块
    BlockImg=srcinput(rect);
    BlockImg_last=BlockImg.clone();
    BlockImg_pre=BlockImg.clone();

    //颜色常量
    lower_blue=Scalar(100-5, 60, 140);
    upper_blue=Scalar(125+5, 255,255);
    lower_green=Scalar(35-5, 60,140);
    upper_green=Scalar(77+5, 255,255);

    lower_red0=Scalar(156, 60,120);
    upper_red0=Scalar(180, 255,255);
    lower_red1=Scalar(0, 60,120);
    upper_red1=Scalar(10, 255,255);
    
}

TrackBlock::~TrackBlock()
{
}

TrackBlock::eTrackStatus TrackBlock::track(Mat srcinput)
{

    BlockImg=srcinput(rect);
    //imshow("BlockImg",BlockImg);

    if(getStatus()==PREPARE)
    {
        char rgbcode;
        //S1：对BlockImg进行hsv识别+轮廓点检测
        vector<Point2f> r_center,g_center,b_center;
        vector<double> SS;
        findContourCenterRGB(BlockImg,r_center,g_center,b_center,SS);
        //S2：找出轮廓点里与中心点最接近的点，计算出移动向量，
        //并且得到该轮廓点是RG还是B（若每找到先退出，下一帧一定能找到）
        double Sarea_avg;
        Vec2f minvec;
        rgbcode=findminVecRGB(r_center,g_center,b_center,SS,minvec,Sarea_avg);
        //cout<<"rgbcode:"<<rgbcode<<endl;
        //S3：根据移动向量更新rect
        //更新跟踪块大小(根据轮廓面积)
        Rect cRect;
        cRect.width=rect.width;
        cRect.height=rect.height;
        mutexLock();
        ledCenter.x+=minvec[0];
        ledCenter.y+=minvec[1];
        cRect.x=ledCenter.x-(rect.width/2);
        cRect.y=ledCenter.y-(rect.height/2);
        mutexUnLock();

        if(setTrackRect(cRect))//看是否越界
        {
            //S4：根据读编码计数规则来更新codeCashe
            //查看是否到了读取时间
        
            if(mpReadCodeImg==mInterval-1)
            {
                mpReadCodeImg=0;
                if(rgbcode=='G')
                {
                    codeCashe[mpReadCodeCNT]=0;
                }
                else if(rgbcode=='B')
                {
                    codeCashe[mpReadCodeCNT]=1;
                }
                else
                {
                    printf("error:读取code错误");
                    codeCashe[mpReadCodeCNT]=-9999;
                }
                mpReadCodeCNT++;
                if(mpReadCodeCNT==codeLength)//读取完成
                {
                    mutexLock();
                    codeID=0;
                    for (size_t i = 0; i < codeLength; i++)
                    {
                        codeID+=codeCashe[i]*pow(2,i);
                    }
                    mutexUnLock();
                    codeStatus=FINISH;
                    rgtbStatus='S';
                    setStatus(OK);
                    mpReadCodeCNT=0;
                }
            }
            else if(mpReadCodeImg<mInterval-1)
                mpReadCodeImg++;
        }
        else
        {
            setStatus(LOSTING);
        }
        


    }
    else if(getStatus()==OK)
    {
        char rgbcode;
        //S1：对BlockImg进行hsv识别+轮廓点检测
        vector<Point2f> r_center,g_center,b_center;
        vector<double> SS;
        findContourCenterRGB(BlockImg,r_center,g_center,b_center,SS);
        //S2：找出轮廓点里与中心点最接近的点，计算出移动向量，
        //并且得到该轮廓点是RG还是B（若每找到先退出，下一帧一定能找到）
        double Sarea_avg;
        Vec2f minvec;
        rgbcode=findminVecRGB(r_center,g_center,b_center,SS,minvec,Sarea_avg);
        //cout<<"OKStatus  rgbcode:"<<rgbcode<<endl;
    
        //S3：根据移动向量更新rect
        //更新跟踪块大小(根据轮廓面积)
        Rect cRect;
        cRect.width=rect.width;
        cRect.height=rect.height;
        mutexLock();
        ledCenter.x+=minvec[0];
        ledCenter.y+=minvec[1];
        cRect.x=ledCenter.x-(rect.width/2);
        cRect.y=ledCenter.y-(rect.height/2);
        mutexUnLock();

         if(setTrackRect(cRect))//看是否越界
        {
            if(codeStatus==FINISH)//这里不需要读取，只需要检测r->G的跳变
            {
                if(rgtbStatus=='S' && rgbcode=='R') rgtbStatus='R';
                else if(rgtbStatus=='R' && rgbcode=='G')
                {
                    rgtbStatus='G';
                    codeStatus=READING;
                    //cout<<"FINISH->READING"<<endl;
                    mpReadCodeImg=mpReadCodeImg_fp;//隔一帧再开始读，保障不读到边缘值
                    mpReadCodeCNT=0;
                }
                else if(rgbcode!='R' && rgbcode!='G' && rgbcode!='B')
                {
                    setStatus(SUSPECTED_LOST);
                    cout<<"SUSPECTED_LOST: FINISH, NO RGB"<<endl;
                }
            }
            else if(codeStatus==READING)
            {
                if(mpReadCodeImg==mInterval-1)
                {
                    if(rgbcode=='G' || rgbcode=='B')
                    {
                        mpReadCodeImg=0;
                        if(rgbcode=='G')
                            codeCashe[mpReadCodeCNT]=0;
                        else if(rgbcode=='B')
                            codeCashe[mpReadCodeCNT]=1;
                        
                        mpReadCodeCNT++;
                        if(mpReadCodeCNT==codeLength)//读取完成
                        {
                            int mcodeID=0;
                            for (size_t i = 0; i < codeLength; i++)
                            {
                                mcodeID+=codeCashe[i]*pow(2,i);
                            }
                            int readIDc=getcodeID();
                            if(mcodeID==readIDc)//ID编码校验成功
                            {
                                codeStatus=FINISH;
                                rgtbStatus='S';
                                setStatus(OK);
                                mpReadCodeCNT=0;
                            }
                            else
                            {
                                setStatus(SUSPECTED_LOST);
                                codeStatus=FINISH;
                                rgtbStatus='S';
                                mpReadCodeCNT=0;
                                cout<<"SUSPECTED_LOST: codeID cant match!"<<endl;
                            }
                            
                            
                            
                        }
                    }
                    else
                    {
                        setStatus(SUSPECTED_LOST);
                        cout<<"SUSPECTED_LOST: READING，NO'G' 'B'"<<endl;
                    }
                    
                }
                else if(mpReadCodeImg<mInterval-1)
                    mpReadCodeImg++;
            }
        }
        else
            setStatus(LOSTING);

    }
    else if(getStatus()==SUSPECTED_LOST)
    {
        char rgbcode;
       //S1：对BlockImg进行hsv识别+轮廓点检测
        vector<Point2f> r_center,g_center,b_center;
        vector<double> SS;
        findContourCenterRGB(BlockImg,r_center,g_center,b_center,SS);
        //S2：找出轮廓点里与中心点最接近的点，计算出移动向量，
        //并且得到该轮廓点是RG还是B（若每找到先退出，下一帧一定能找到）
        double Sarea_avg;
        Vec2f minvec;
        rgbcode=findminVecRGB(r_center,g_center,b_center,SS,minvec,Sarea_avg);
        cout<<"SUSPECTED_LOST  rgbcode:"<<rgbcode<<endl;
    
        //S3：根据移动向量更新rect
        //更新跟踪块大小(根据轮廓面积)
        Rect cRect;
        cRect.width=rect.width;
        cRect.height=rect.height;
        mutexLock();
        ledCenter.x+=minvec[0];
        ledCenter.y+=minvec[1];
        cRect.x=ledCenter.x-(rect.width/2);
        cRect.y=ledCenter.y-(rect.height/2);
        mutexUnLock();

        if(setTrackRect(cRect))//看是否越界
        {
            if(codeStatus==FINISH)//这里不需要读取，只需要检测r->G的跳变
            {
                if(rgtbStatus=='S' && rgbcode=='R')
                {   
                    rgtbStatus='R';
                    setStatus(OK);
                }
                else if(rgtbStatus=='R' && rgbcode=='G')
                {
                    rgtbStatus='G';
                    codeStatus=READING;
                    cout<<"FINISH->READING"<<endl;
                    mpReadCodeImg=mpReadCodeImg_fp;
                    mpReadCodeCNT=0;
                    setStatus(OK);
                }
                else if(rgbcode!='R' && rgbcode!='G' && rgbcode!='B')
                {
                    setStatus(LOSTING);
                    rgtbStatus=NULL;
                    cout<<"LOST:FINISH ;NO RGB"<<endl;
                }
            }
            else if(codeStatus==READING)
            {
                if(mpReadCodeImg==mInterval-1)
                {
                    if(rgbcode=='G' || rgbcode=='B')
                    {
                        mpReadCodeImg=0;
                        setStatus(OK);
                        if(rgbcode=='G')
                        {
                            codeCashe[mpReadCodeCNT]=0;
                        }
                        else if(rgbcode=='B')
                        {
                            codeCashe[mpReadCodeCNT]=1;
                        }
                        
                        mpReadCodeCNT++;
                        if(mpReadCodeCNT==codeLength)//读取完成
                        {
                            int mcodeID=0;
                            for (size_t i = 0; i < codeLength; i++)
                            {
                                mcodeID+=codeCashe[i]*pow(2,i);
                            }
                            int readIDc=getcodeID();
                            if(mcodeID==readIDc)//ID编码校验成功
                            {
                                codeStatus=FINISH;
                                rgtbStatus='S';
                                setStatus(OK);
                                mpReadCodeCNT=0;
                                mpReadCodeImg=0;
                            }
                            else
                            {
                                setStatus(LOSTING);
                                cout<<"LOST: codeID cant match!"<<endl;
                            }
                            
                        }
                    }
                    else
                    {
                        setStatus(LOSTING);
                        rgtbStatus=NULL;
                        cout<<"LOST :READING ;NO'G' 'B'"<<endl;
                    }
                    
                }
                else if(mpReadCodeImg<mInterval-1)
                {
                    if(rgbcode=='G' || rgbcode=='B')setStatus(OK);
                    mpReadCodeImg++;
                }
            }
        }
        else
            setStatus(LOSTING);

    }
    else if(getStatus()==LOSTING)
    {
        codeStatus=FINISH;
        rgtbStatus='S';
    }
    


    BlockImg_pre=BlockImg_last.clone();
    BlockImg_last=BlockImg.clone();
}


void TrackBlock::findContourCenterRGB(Mat inputImg ,vector<Point2f> &r,vector<Point2f> &g,vector<Point2f> &b,vector<double> &Sarea)
{
    //转换hsv
    Mat img_hsv,mask_g,mask_b,mask_r;
    cvtColor(inputImg,img_hsv,COLOR_BGR2HSV);

    inRange(img_hsv,lower_green,upper_green,mask_g);
    inRange(img_hsv,lower_blue,upper_blue,mask_b);
    Mat mask_r0,mask_r1;
    inRange(img_hsv,lower_red0,upper_red0,mask_r0);
    inRange(img_hsv,lower_red1,upper_red1,mask_r1);
    bitwise_or(mask_r0,mask_r1,mask_r);


    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //r
    vector<Point2f> rr;
    cv::findContours(mask_r, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
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
            rr.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    r=rr;
    //g
    vector<Point2f> gg;
    cv::findContours(mask_g, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
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
            gg.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    g=gg;
    //b
    vector<Point2f> bb;
    cv::findContours(mask_b, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
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
            bb.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    b=bb;
    return ;

}

void TrackBlock::findContourCenterGB(Mat inputImg ,vector<Point2f> &g,vector<Point2f> &b,vector<double> &Sarea)
{
    //转换hsv
    Mat img_hsv,mask_g,mask_b,mask_r;
    cvtColor(inputImg,img_hsv,COLOR_BGR2HSV);

    inRange(img_hsv,lower_green,upper_green,mask_g);
    inRange(img_hsv,lower_blue,upper_blue,mask_b);
    

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //g
    vector<Point2f> gg;
    cv::findContours(mask_g, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
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
            gg.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    g=gg;
    //b
    vector<Point2f> bb;
    cv::findContours(mask_b, contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
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
            bb.push_back(Point2f(cx,cy));
            Sarea.push_back(S);
        }
    }
    b=bb;
}


char TrackBlock::findminVecRGB(vector<Point2f> r,vector<Point2f> g,vector<Point2f> b,vector<double> Sarea,Vec2f &OutputminVec,double &OutputSarea)
{
    double minvecjuli=999999;
    Vec2f minvec;
    char rgbcode=NULL;
    double okSarea;
    
    for(int i=0;i<g.size();i++)
    {
        Vec2f vecoi;
        double vecjuli;
        //vecoi=g[i]-Point2f(rect.width/2,rect.height/2);
        vecoi[0]=g[i].x-rect.width/2;
        vecoi[1]=g[i].y-rect.height/2;
        vecjuli=sqrt(vecoi[0]*vecoi[0]+vecoi[1]*vecoi[1]);
        if (vecjuli<minvecjuli)
        {
            minvecjuli=vecjuli;
            minvec=vecoi;
            rgbcode='G';
            okSarea=Sarea[r.size()+i];
        }
    }
    
    for(int i=0;i<b.size();i++)
    {
         Vec2f vecoi;
        double vecjuli;
        //vecoi=g[i]-Point2f(rect.width/2,rect.height/2);
        vecoi[0]=b[i].x-rect.width/2;
        vecoi[1]=b[i].y-rect.height/2;
        vecjuli=sqrt(vecoi[0]*vecoi[0]+vecoi[1]*vecoi[1]);
        if (vecjuli<minvecjuli)
        {
            minvecjuli=vecjuli;
            minvec=vecoi;
            rgbcode='B';
            okSarea=Sarea[r.size()+g.size()+i];
        }
    }
    for(int i=0;i<r.size();i++)
    {
         Vec2f vecoi;
        double vecjuli;
        //vecoi=g[i]-Point2f(rect.width/2,rect.height/2);
        vecoi[0]=r[i].x-rect.width/2;
        vecoi[1]=r[i].y-rect.height/2;
        vecjuli=sqrt(vecoi[0]*vecoi[0]+vecoi[1]*vecoi[1]);
        if (vecjuli<minvecjuli)
        {
            minvecjuli=vecjuli;
            minvec=vecoi;
            rgbcode='R';
            okSarea=Sarea[i];
        }
    }
    
    //校验速度，因为变化不可能太快
    //通过加速度来判别
    Point2f acc;
    acc.x=minvec[0]-mVelocity.x;
    acc.y=minvec[1]-mVelocity.y;
    double accmod=sqrt(acc.dot(acc));
    if(accmod<rect.width*0.3)
    {
        OutputminVec=minvec;
        OutputSarea=okSarea;
        mVelocity=Point2f(minvec[0],minvec[1]);
    }
    else
    {
        cout<<"加速度过大，当前帧失去跟踪点"<<endl;
        OutputminVec=Vec2f(0,0);
        OutputSarea=-1;
        mVelocity=Point2f(0,0);
        rgbcode=NULL;
    }
    

    return rgbcode;
}


Point2f TrackBlock::getCenter()
{
    unique_lock<mutex> lock(*pSysMutex);
    return ledCenter;
}
int TrackBlock::getcodeID()
{
    unique_lock<mutex> lock(*pSysMutex);
    return codeID;
}
Rect TrackBlock::getTrackRect()
{
    unique_lock<mutex> lock(*pSysMutex);
    return rect;
}
TrackBlock::eTrackStatus TrackBlock::getStatus()
{
    unique_lock<mutex> lock(*pSysMutex);
    return TrackStatus;
}
//设置数据
void TrackBlock::setCenter(Point2f p)
{
   unique_lock<mutex> lock(*pSysMutex);
    ledCenter=p;
}
void TrackBlock::setcodeID(int id)
{
    unique_lock<mutex> lock(*pSysMutex);
    codeID=id;
}
void TrackBlock::setStatus(eTrackStatus status)
{
    unique_lock<mutex> lock(*pSysMutex);
    TrackStatus=status;
}

bool TrackBlock::setTrackRect(Rect rt)
{
    unique_lock<mutex> lock(*pSysMutex);
    //做评估
    if(rt.x>0&&rt.y>0&& (rt.x+rt.width<srcImgSize.width-1)&&(rt.y+rt.height<srcImgSize.height-1))
    {
        rect=rt;
        return true;
    }
    else
    {
        printf("setTrackRect: roi 不满足要求");
        return false;
    }
        
}

void TrackBlock::mutexLock()
{
    pSysMutex->lock();
}
void TrackBlock::mutexUnLock()
{
    pSysMutex->unlock();
}


}
