#include "GeomeOperation.h"

namespace LED_POSITION
{

bool geomeopera::IntersectionOf2Circles(CIRCLE c1, CIRCLE c2, cv::Point2f &P1, cv::Point2f &P2)
{
	float a1, b1, R1, a2, b2, R2;
	a1 = c1.c.x;
	b1 = c1.c.y;
	R1 = c1.r;

	a2 = c2.c.x;
	b2 = c2.c.y;
	R2 = c2.r;

	//
	float R1R1 = R1*R1;
	float a1a1 = a1*a1;
	float b1b1 = b1*b1;

	float a2a2 = a2*a2;
	float b2b2 = b2*b2;
	float R2R2 = R2*R2;

	float subs1 = a1a1 - 2 * a1*a2 + a2a2 + b1b1 - 2 * b1*b2 + b2b2;
	float subs2 = -R1R1 * a1 + R1R1 * a2 + R2R2 * a1 - R2R2 * a2 + a1a1*a1 - a1a1 * a2 - a1*a2a2 + a1*b1b1 - 2 * a1*b1*b2 + a1*b2b2 + a2a2*a2 + a2*b1b1 - 2 * a2*b1*b2 + a2*b2b2;
	float subs3 = -R1R1 * b1 + R1R1 * b2 + R2R2 * b1 - R2R2 * b2 + a1a1*b1 + a1a1 * b2 - 2 * a1*a2*b1 - 2 * a1*a2*b2 + a2a2 * b1 + a2a2 * b2 + b1b1*b1 - b1b1 * b2 - b1*b2b2 + b2b2*b2;
	float sigma = sqrt((R1R1 + 2 * R1*R2 + R2R2 - a1a1 + 2 * a1*a2 - a2a2 - b1b1 + 2 * b1*b2 - b2b2)*(-R1R1 + 2 * R1*R2 - R2R2 + subs1));
	if(abs(subs1)>0.0000001)//分母不为0
	{
		P1.x = (subs2 - sigma*b1 + sigma*b2) / (2 * subs1);
		P2.x = (subs2 + sigma*b1 - sigma*b2) / (2 * subs1);
	
		P1.y = (subs3 + sigma*a1 - sigma*a2) / (2 * subs1);
		P2.y = (subs3 - sigma*a1 + sigma*a2) / (2 * subs1);
        return true;
	}
    return false;

}

g2o::SE2 geomeopera::estimpose(vector<Eigen::Vector2d> z,vector<Eigen::Vector2d> p,g2o::SE2 pose0)
{

	// 构建图优化，先设定g2o
    // 矩阵块：每个误差项优化变量维度为4 ，误差值维度为1
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,2>> Block;
	// 线性方程求解器：稠密的增量方程
	std::unique_ptr<Block::LinearSolverType> linearSolver( new g2o::LinearSolverDense<Block::PoseMatrixType>());
	std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));//矩阵块求解器
	//梯度下降方法LM
	g2o::OptimizationAlgorithmLevenberg* solver
		 = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );

	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);//设置求解器
	optimizer.setVerbose(false);//打开调试输出

	// 往图中增加顶点
    g2o::VertexSE2 *v = new g2o::VertexSE2();
	//设置优化初始值
	v->setEstimate(pose0);
	v->setId(0);
	v->setFixed(false);
	optimizer.addVertex(v);

	//往图中增加边
	for(int i=0;i<z.size();i++)
	{
		EdgePointToSE2* edge =new EdgePointToSE2();
		edge->setId(i+1);
		edge->setVertex(0,v);
		edge->setMeasurement(z[i]);
		// 信息矩阵：协方差矩阵之逆
        edge->setInformation ( Eigen::Matrix2d::Identity() );
		edge->p=p[i];
		optimizer.addEdge(edge);

	}
	
	optimizer.initializeOptimization();
    optimizer.optimize(20);
	
	return v->estimate();

	
	
	
}

geomeopera::geomeopera(/* args */)
{

}

geomeopera::~geomeopera()
{
}

}//name