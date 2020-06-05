#include <iostream>
#include <fstream>

#include "backend/problem.h"
#include "backend/vertex.h"
#include "backend/edge.h"
#include "utils/tic_toc.h"

using namespace std;

Problem::Problem(ProblemType problemType) : problemType_(problemType)
{

}

Problem::~Problem()
{

}

bool Problem::AddVertex(std::shared_ptr<Vertex> vertex)
{
    if (vertices_.find(vertex->Id()) == vertices_.end())
    {
        vertices_.insert(pair<unsigned long, shared_ptr<Vertex>> (vertex->Id(), vertex));
        return true;
    }
    else
        return false;
}

bool Problem::AddEdge(std::shared_ptr<Edge> edge)
{
    if (edges_.find(edge->Id()) == edges_.end())
        edges_.insert(pair<unsigned long, shared_ptr<Edge>> (edge->Id(), edge));
    else
        return false;
    
    for (auto &vertex : edge->GetVertices())
    {
        vertexToEdge_.insert(pair<unsigned long, shared_ptr<Edge>> (vertex->Id(), edge));
    }

    return true;
}
    
bool Problem::Solve(int iterations)
{
    if (edges_.size() == 0 || vertices_.size() == 0)
    {
        cerr << "Cannot solve problem without edges and vertices" << endl;
        return false;
    }

    TicToc t_solve;

    // 统计优化变量的维度，为构建Hessian矩阵做准备
    SetOrdering();

    // 构建Hessian矩阵，增量正规方程
    MakeHessian();

    // LM算法初始化，选取初始阻尼因子lambda
    ComputeLambdaInitLM();

    // 开始迭代求解
    bool stop = false;
    int iter = 0;
    while (!stop && (iter < iterations))
    {
        cout << "Iteration = " << iter << ", chi2 = " << currentChi2_ << ", Lambda = " << currentLambda_ << endl;
        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess)  // 不断尝试lambda，直到成功迭代一步
        {
            // 向增量方程添加lambda
            AddLambdaHessianLM();
            // 解线性方程 HX = b
            SolveLinearSystem();
            // 移除lambda，便于下次更新lambda后再添加
            RemoveLambdaHessianLM();

            // 判断是否停止迭代更新
            // cout << delta_x_.squaredNorm() << endl;
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10)  // squareNorm：二范数
            {
                stop = true;
                break;
            }

            // 更新变量，x' = x + delta_x_
            UpdateStates();
            // 判断当前更新的可靠性以及阻尼因子lambda的更新
            oneStepSuccess = IsGoodStepInLM();
            if (oneStepSuccess)
            {
                // 在新线性化点构建Hessian矩阵
                MakeHessian();
                false_cnt = 0;
            }
            else
            {
                false_cnt++;
                // 误差没有减小，回滚
                RollbackStates();
            }
        }
        iter++;
        if (sqrt(currentChi2_) <= stopLMThreshold_)  // 迭代条件：误差小于初始误差的1e-6倍
            stop = true;
    }

    cout << "make hessian matrix cost: " << time_hessian_ << " ms." << endl;
    cout << "Non-linear optimization solve cost: " << t_solve.toc() << " ms." << endl;
    
}

void Problem::SetOrdering()
{
    // 每次先重新计数
    ordering_poses_ = 0;
    ordering_landmarks_ = 0;
    ordering_generic_ = 0;

    for (auto vertex : vertices_)
    {
        ordering_generic_ += vertex.second->LocalDimension();
    }
}

void Problem::MakeHessian()
{
    TicToc t_hessian;

    unsigned long N = ordering_generic_;
    MatXX H(MatXX::Zero(N, N));
    VecX b(VecX::Zero(N));

    for (auto &edge : edges_)
    {
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians_optim = edge.second->GetJacobians();
        auto vertices_optim = edge.second->GetVertices();
        assert(jacobians_optim.size() == vertices_optim.size());
        for (size_t i = 0; i < vertices_optim.size(); i++)
        {
            auto v_i = vertices_optim[i];
            // Hessian矩阵里不需要添加它的信息，也就是雅克比矩阵
            if (v_i->IsFixed())
                continue;

            auto jacobian_i = jacobians_optim[i];
            unsigned long index_i = v_i->OrderingId();
            unsigned long dim_i = v_i->LocalDimension();

            for (size_t j = i; j < vertices_optim.size(); j++)
            {
                auto v_j = vertices_optim[j];
                if (v_j->IsFixed())
                    continue;

                auto jacobian_j = jacobians_optim[j];
                unsigned long index_j = v_j->OrderingId();
                unsigned long dim_j = v_j->LocalDimension();

                assert(index_j != -1);

                // H = JT * sigma * J
                MatXX hessian = jacobian_i.transpose() * edge.second->GetInformation() * jacobian_j;
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;  // noalias():确保矩阵运算中不会出现混淆问题

                if (j != i)
                {
                    // 对称的下三角
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }

            // b = -JT * sigma * residual
            b.segment(index_i, dim_i).noalias() -= jacobian_i.transpose() * edge.second->GetInformation() * edge.second->GetResidual();
            
        }
        
    }

    H_ = H;
    b_ = b;
    delta_x_ = VecX::Zero(N);
    time_hessian_ += t_hessian.toc();
}

void Problem::ComputeLambdaInitLM()
{
    for (auto edge : edges_)
        currentChi2_ += edge.second->Chi2();
    // if (err_prior_.rows() > 0)
    //     currentChi2_ += err_prior_.norm();
    
    stopLMThreshold_ = 1e-6 * currentChi2_;
    cout << "StopLMThreshold_ = " << stopLMThreshold_ << endl;

    assert(H_.rows() == H_.cols() && "Hessian matrix is not square!");
    double maxDiagonal = 0.0;  // 表示Hessian矩阵中，对角线元素的最大值
    for (unsigned long i = 0; i < H_.cols(); i++)
    {
        maxDiagonal = max(fabs(H_(i, i)), maxDiagonal);
    }

    double tau = 1e-5;  // 取值范围：[1e-8, 1]
    currentLambda_ = tau*maxDiagonal;
}


void Problem::AddLambdaHessianLM()
{
    assert(H_.rows() == H_.cols() && "Hessian matrix is not square!");
    H_without_lambda_ = H_;
    for (unsigned long i = 0; i < H_.cols(); i++)
    {
        H_(i, i) += currentLambda_;
    }
    // cout << "H_without_lambda_:\n" << H_without_lambda_ << endl;
    // cout << "H_:\n" << H_ << endl;
}

void Problem::RemoveLambdaHessianLM()
{
    assert(H_.rows() == H_.cols() && "Hessian matrix is not square!");
    H_ = H_without_lambda_;
}

void Problem::SolveLinearSystem()
{
    delta_x_ = H_.inverse() * b_;
    // delta_x_ = H_.ldlt().solve(b_);
    // cout << "delta_x_.transpose = " << delta_x_.transpose() << endl;
}

void Problem::UpdateStates()
{
    for (auto vertex : vertices_)
    {
        unsigned long idx = vertex.second->OrderingId();
        unsigned long dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);
        vertex.second->Plus(delta);
    }
    
}

void Problem::RollbackStates()
{
    for (auto vertex : vertices_)
    {
        unsigned long idx = vertex.second->OrderingId();
        unsigned long dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);
        // 之前加了增量使目标函数的误差增加，因此不需要这次的迭代结果
        vertex.second->Plus(-delta);
    }   
}

bool Problem::IsGoodStepInLM()
{
    double grad_descent_obs = 0.0;  // 实际梯度下降值
    double grad_descent_pred = 0.0;  // 近似梯度下降值
    
    double tempChi2 = 0.0;
    for (auto edge : edges_)
    {
        edge.second->ComputeResidual();  // 此时已添加增量，重新计算新的残差
        tempChi2 += edge.second->Chi2();
    }
    // cout << "old = " << currentChi2_ << " new = " << tempChi2 << endl;
    grad_descent_obs = currentChi2_ - tempChi2;  // 旧误差 - 新误差
    grad_descent_pred = delta_x_.transpose() * (currentLambda_*delta_x_ + b_);
    double rho = grad_descent_obs / (grad_descent_pred + 1e-3);  // 1e-3确保分母不为0

    // 采用Nielsen阻尼更新策略
    if (rho > 0 && isfinite(tempChi2))  // 残差减小，减小阻尼，使之更快收敛
    {
        double alpha = 1 - pow((2*rho - 1), 3);
        alpha = min(alpha, 2.0/3.0);
        currentLambda_ *= max(1.0/3.0, alpha);
        currentChi2_ = tempChi2;

        return true;
    }
    else  // 残差增大，增大阻尼，减小迭代步长
    {
        currentLambda_ *= ni_;
        ni_ *= 2;
        
        return false;
    }
    
}


