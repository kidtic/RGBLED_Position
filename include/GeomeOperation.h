//几何运算库，在定位中可能会遇到的几何运算
#ifndef  GEOLOCATION_H
#define  GEOLOCATION_H
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/edge_se2.h>


using namespace std;

namespace LED_POSITION
{

class geomeopera
{
public:
    typedef struct
    {
        cv::Point2f c;
        float r;
        void CIRCLE(cv::Point2f cin, float rin){
            c=cin;
            r=rin;
        };
    }CIRCLE;

private:
    /* data */
    

public:
    geomeopera(/* args */);
    ~geomeopera();

    //求两圆交点
    bool IntersectionOf2Circles(CIRCLE c1, CIRCLE c2, cv::Point2f &P1, cv::Point2f &P2);
    //二维平面姿态估计--坐标变换匹配
    //z:观察到的相对世界坐标系的点
    //p:定义的相对于机器人坐标系的定位点，
    //机器人的初始化位姿
    //返回优化后的位姿。
    g2o::SE2 estimpose(vector<Eigen::Vector2d> z,vector<Eigen::Vector2d> p,g2o::SE2 pose0);

    

};



// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class EdgePointToSE2: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    bool read(std::istream& is) {}
    bool write(std::ostream& os) const {}

    void computeError()  {
        const g2o::VertexSE2* pose = static_cast<const g2o::VertexSE2*> ( _vertices[0] );
        _error = _measurement - (pose->estimate().rotation().matrix()*p+pose->estimate().translation());
    }

    virtual void linearizeOplus(){
        g2o::VertexSE2* pose = static_cast<g2o::VertexSE2*> ( _vertices[0] );  

        double theta=pose->estimate().toVector()[2];
        double sintheta=sin(theta);
        double costheta=cos(theta);
        double xi=p[0];
        double yi=p[1];

        _jacobianOplusXi ( 0,0 ) = -1;
        _jacobianOplusXi ( 0,1 ) = 0;
        _jacobianOplusXi ( 0,2 ) = xi*sintheta+yi*costheta;


        _jacobianOplusXi ( 1,0 ) = 0;
        _jacobianOplusXi ( 1,1 ) = -1;
        _jacobianOplusXi ( 1,2 ) = -xi*costheta+yi*sintheta;

    }

    Eigen::Vector2d p;
};










}//namespace LED_POSITION



#endif