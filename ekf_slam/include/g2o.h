/*
 * @Author: fengkai 
 * @Date: 2019-07-18 14:38:43 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-18 21:18:31
 */
#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

//曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;
    }
    
    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x,double y): BaseUnaryEdge(), _x(x),_y(y) {}
    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d xydelta = v->estimate();
        Eigen::Vector2d obs(_measurement);
        _error(0,0) = obs(0,0) - (_x*std::cos(xydelta(2,0)-_y*std::sin(xydelta(2,0))+xydelta(0,0)));
        _error(1,0) = obs(1,0) - (_x*std::sin(xydelta(2,0)+_y*std::cos(xydelta(2,0))+xydelta(1,0)));

    }
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
public:
    double _x,_y;  // x 值， y 值为 _measurement
};