/*
 * @Author: fengkai 
 * @Date: 2019-07-18 14:38:51 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-18 21:18:10
 */

#include "g2o.h"

int main( int argc, char** argv )
{
    double a=1, b=2, c=1.57;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=0.1;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double xyd[3] = {0,0,0};            // abc参数的估计值

    std::vector<Eigen::Vector2d> in;      // 数据
    std::vector<Eigen::Vector2d> out;     // 数据

    
    std::cout<<"generating data: "<<std::endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i;
        double y = i*2;
        in.push_back(Eigen::Vector2d(x,y));
        
        out.push_back(Eigen::Vector2d(x*std::cos(c)-y*std::sin(c)+a,
                                      x*std::sin(c)+y*std::cos(c)+b));
        std::cout<<in[i]<<std::endl<<out[i]<<std::endl;
    }
    
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,2> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
    
    //第一步：创建线性方程求解器
    std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverDense<Block::PoseMatrixType>()); // 线性方程求解器
    
    //第二步：创建BlockSolver，并用上面定义的线性求解器初始化
    std::unique_ptr<Block> solver_ptr (new Block( std::move(linearSolver)));      // 矩阵块求解器
    
    // 第三步：创建总求解器solver，梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move( solver_ptr));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    
    //第四步：创建稀疏优化器SparseOptimizer
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // 打开调试输出

    
    // 第五步：定义图的顶点和边，往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge(in.at(i)(0,0),in.at(i)(1,0));
        edge->setId(i);
        edge->setVertex( 0, v );                // 设置连接的顶点
        edge->setMeasurement( out[i] );      // 观测数值
        edge->setInformation( Eigen::Matrix<double,2,2>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge( edge );
    }
    
    // 第六步：设置优化参数，开始执行优化
    std::cout<<"start optimization"<<std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
    std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout<<"estimated model: "<<abc_estimate.transpose()<<std::endl;
    
    return 0;
}