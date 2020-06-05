#include <iostream>
#include <bits/stdc++.h>  // 解决 error: ‘random_device’ or 'default_random_engine' is not a member of ‘std’ !!!
#include "backend/problem.h"
#include "backend/vertex.h"
#include "backend/edge.h"

using namespace std;


// 待优化变量构成顶点
class CurveFittingVertex : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CurveFittingVertex() : Vertex(3) {}  // 输入的优化项是a,b,c 3个参数，因此Vertex是3维的向量
    virtual std::string TypeInfo() const { return "abc"; }
};


// 残差值构成边
class CurveFittingEdge : public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CurveFittingEdge(double x, double y) : Edge(1, 1, std::vector<std::string> {"abc"})
    {
        x_ = x;
        y_ = y;
    }

    // 计算曲线模型的残差值
    virtual void ComputeResidual() override
    {
        Vec3 abc = vertices_[0]->GetParameters();
        // residual_[0] = exp(abc[0]*x_*x_ + abc[1]*x_ + abc[2]) - y_;  // 预测值 - 观测值
        residual_[0] = abc[0]*x_*x_ + abc[1]*x_ + abc[2] - y_;
    }

    // 计算曲线模型的残差值对变量a,b,c的雅克比
    virtual void ComputeJacobians() override
    {
        Vec3 abc = vertices_[0]->GetParameters();
        // double y_pred = exp(abc[0]*x_*x_ + abc[1]*x_ + abc[2]);
        double y_pred = abc[0]*x_*x_ + abc[1]*x_ + abc[2];

        Mat13 J_r_abc;  // 误差1维，状态量3个，所以是1*3的雅克比矩阵
        // J_r_abc << y_pred*x_*x_, y_pred*x_, y_pred;
        J_r_abc << x_*x_, x_, 1;
        jacobians_[0] = J_r_abc;
    }

    // 返回边的类型信息
    virtual std::string TypeInfo() const override
    {
        return "CurveFittingEdge";
    }

public:
    double x_, y_;  // 观测值

};



int main(int argc, char const *argv[])
{
    // 估计曲线 y = exp(a*x*x + b*x + c)；y = a*x*x + b*x + c
    // double a = 1.0, b = 2.0, c = 1.0;
    double a = 10.0, b = 20.0, c = 10.0;
    int N = 100;
    double w_sigma = 1.0;  // 噪声sigma值
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, w_sigma);

    // 构建最小二乘优化问题
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr<CurveFittingVertex> vertex(new CurveFittingVertex());
    vertex->SetParameters(Eigen::Vector3d(0.0, 0.0, 0.0));  // 优化变量的初始值

    // 将待优化项加入到优化问题
    problem.AddVertex(vertex);
    // 构造N次观测
    for (int i = 0; i < N; i++)
    {
        double x = i/100.0;
        double n = noise(generator);
        // double y = exp(a*x*x + b*x + c) + n;
        double y = a*x*x + b*x + c + n;

        // 每个观测对应的残差函数
        shared_ptr<CurveFittingEdge> edge(new CurveFittingEdge(x, y));
        vector<shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        // 把残差添加到优化问题中
        problem.AddEdge(edge);
    }

    cout << "Start Optimizing ..." << endl;

    // 使用L-M法求解，迭代30次
    problem.Solve(30);

    // cout << "For function y=exp(a*x*x+b*x+c):\n";
    // cout << "After optmization, we got parameters: " << vertex->GetParameters().transpose()
    //      << "\nGround truth: 1.0, 2.0, 1.0" << endl;
    
    /*
    Iteration = 0, chi2 = 36048.3, Lambda = 0.001
    Iteration = 1, chi2 = 30015.5, Lambda = 699.051
    Iteration = 2, chi2 = 29225.9, Lambda = 29826.2
    Iteration = 3, chi2 = 26303.6, Lambda = 9942.05
    Iteration = 4, chi2 = 11894.7, Lambda = 3314.02
    Iteration = 5, chi2 = 2452.21, Lambda = 2045.47
    Iteration = 6, chi2 = 194.609, Lambda = 681.824
    Iteration = 7, chi2 = 111.868, Lambda = 227.275
    Iteration = 8, chi2 = 102.784, Lambda = 75.7582
    Iteration = 9, chi2 = 98.3442, Lambda = 25.2527
    Iteration = 10, chi2 = 93.5904, Lambda = 8.41758
    Iteration = 11, chi2 = 91.6152, Lambda = 2.80586
    Iteration = 12, chi2 = 91.4003, Lambda = 0.935287
    Iteration = 13, chi2 = 91.3959, Lambda = 0.623525
    make hessian matrix cost: 2.38227 ms.
    Non-linear optimization solve cost: 2.94842 ms.
    After optmization, we got parameters:  0.94233  2.09396 0.965781
    Ground truth: 1.0, 2.0, 1.0 
     */

    cout << "For function y=a*x*x+b*x+c:\n";
    cout << "After optmization, we got parameters: " << vertex->GetParameters().transpose()
         << "\nGround truth: 10, 20, 10" << endl;
    
    /*
    Iteration = 0, chi2 = 61493.7, Lambda = 0.001
    Iteration = 1, chi2 = 91.3952, Lambda = 0.000333333
    Iteration = 2, chi2 = 91.395, Lambda = 0.000222222
    make hessian matrix cost: 0.540315 ms.
    Non-linear optimization solve cost: 0.659366 ms.
    For function y=a*x*x+b*x+c:
    After optmization, we got parameters: 10.6107 19.6183 9.99517
    Ground truth: 10, 20, 10
     */

    return 0;
}
