#include "GeomeOperation.h"
#include <opencv2/core.hpp>
#include <iostream>


int main(int argc, char const *argv[])
{
    LED_POSITION::geomeopera hp;

    
    vector<Eigen::Vector2d> p;
    p.push_back(Eigen::Vector2d(1,1));
    p.push_back(Eigen::Vector2d(1,-1));
    g2o::SE2 pose(6.35434,-5.23526,1.756);
    vector<Eigen::Vector2d> z;
    z.push_back(pose*p[0]); 
    z.push_back(pose*p[1]);
    g2o::SE2 pose0(0,0,0);

    g2o::SE2 res = hp.estimpose(z,p,pose0);
    cout<<"res=\n"<<res.toVector()<<endl;






    return 0;
}
