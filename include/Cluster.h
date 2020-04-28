/*******************
 * 自定义的聚类算法 
 */
#ifndef  CLUSTER_H
#define CLUSTER_H

#include<opencv2/core.hpp>
#include<vector>

using namespace cv;
using namespace std;

namespace LED_POSITION
{

class Cluster
{
private:
    vector<Point2f> listpoint;//要聚类的数据点
    vector<int> classidcount; //记录CLASSID号的个数，索引代表classid号
    vector<int> classid; //listpoint所对应的CLASSID号，所以size要与listpoint一样
    int classNum;   //聚类得到的数目

    //中心点
    vector<Point2f> center;
    vector<int> centerID; //center所对应的classID号
    //参数
    double esp;  //感染领域
    int minClassNum;  //最小类数
public:
    Cluster();
    ~Cluster();

    //输入数据
    void initdata(vector<Point2f> dataInput,double esps);
    //聚类
    void clustering();

    //返回中心点与其ID号
    void getCenter(vector<Point2f> &OutputCenter,vector<int> &OutputCenterID);
    void getCenter(vector<Point2f> &OutputCenter);
    vector<int> getDataID();


    //判断元素首否在vecter里,返回索引
    int findVec_int(vector<int> a,int b);
    
};





}//LED_POSITION



#endif