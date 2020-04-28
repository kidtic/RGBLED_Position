/*******************
 * 自定义的聚类算法 
 */
#include"Cluster.h"


namespace LED_POSITION
{

Cluster::Cluster(/* args */)
{
}

Cluster::~Cluster()
{
}

void Cluster::initdata(vector<Point2f> dataInput,double esps)
{
    //清楚之前的记录
    classidcount.clear();
    classid.clear();
    classNum=0;
    center.clear();
    centerID.clear();

    listpoint=dataInput;
    esp=esps;
    for (size_t i = 0; i < listpoint.size(); i++)
    {
        classidcount.push_back(1);
        classid.push_back(i);
    }
}

void Cluster::clustering()
{
    //组合数，计算两两之间的距离
    vector<Vec2i> listpoint_connect;//需要聚类的点
    double clusteresp=esp;  //领域距离
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

    vector<Point2f> rg_centers_c;
    vector<int> rg_centers_c_count;
    
    for(int i=0;i<listpoint.size();i++)
    {
        int clss=classid[i];
        //判断这个id是否在centerID里
        int index=findVec_int(centerID,clss);
        if(index==-1)
        {
            centerID.push_back(clss);
            rg_centers_c.push_back(Point2f(0,0));
            rg_centers_c_count.push_back(0);
            index=centerID.size()-1;
        }
        //按照索引来加
        rg_centers_c_count[index]++;
        rg_centers_c[index]+=listpoint[i];
    }
    classNum=rg_centers_c.size();

    for(uint j=0;j<classNum;j++)
    {
        rg_centers_c[j]=rg_centers_c[j]/rg_centers_c_count[j];
    }
    center=rg_centers_c;//更新center变量

}

 //返回中心点与其ID号
void Cluster::getCenter(vector<Point2f> &OutputCenter,vector<int> &OutputCenterID)
{
    OutputCenter=center;
    OutputCenterID=centerID;
}
void Cluster::getCenter(vector<Point2f> &OutputCenter)
{
    OutputCenter=center;
}


int Cluster::findVec_int(vector<int> a,int b)
{
    int i=0;
    for(;i<a.size();i++)
    {
        if(a[i]==b) return i;
    }
    return -1;
}

vector<int> Cluster::getDataID()
{
    return classid;
}

}
