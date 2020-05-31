#include "GeomeOperation.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <iterator>
#include <random>


int main(int argc, char const *argv[])
{
    LED_POSITION::geomeopera hp;

    
    vector<Eigen::Vector2d> p;
    p.push_back(Eigen::Vector2d(0.1579,0.01995));
    p.push_back(Eigen::Vector2d(-0.08532,0.13435));
    p.push_back(Eigen::Vector2d(-0.08532,-0.13435));
    g2o::SE2 pose(1,1,3.14);
    // Define random generator with Gaussian distribution
    const double mean = 0.0;//均值
    const double stddev = 0.00001;//标准差
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    

    while(1)
    {
        vector<Eigen::Vector2d> z;
        z.push_back(pose*p[0]+Eigen::Vector2d(dist(generator),dist(generator))); 
        z.push_back(pose*p[1]+Eigen::Vector2d(dist(generator),dist(generator)));
        z.push_back(pose*p[2]+Eigen::Vector2d(dist(generator),dist(generator)));
        //z.push_back(Eigen::Vector2d(-0.00584769 , 0.635178)+Eigen::Vector2d(dist(generator),dist(generator)));
        //z.push_back(Eigen::Vector2d(-0.1195 , 0.380896)+Eigen::Vector2d(dist(generator),dist(generator)));
        //z.push_back(Eigen::Vector2d(0.155082 , 0.386187)+Eigen::Vector2d(dist(generator),dist(generator)));
        g2o::SE2 pose0(0,0,0);
        g2o::SE2 res = hp.estimpose(z,p,pose0);
        cout<<"res=\n"<<res.toVector()<<endl;
        char a;
        cin>>a;
    }

    






    return 0;
}
