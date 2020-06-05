#ifndef PROBLEM_H
#define PROBLEM_H

#include <map>
#include <unordered_map>
#include <memory>

#include "backend/eigen_types.h"
#include "backend/vertex.h"
#include "backend/edge.h"

class Problem
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 要注意非线性优化方法求解的是SLAM问题还是通用问题
     * 如果是SLAM问题，pose和landmark是区分开的，Hessian以稀疏结构存储，而且只接受一些特定的vertex和edge
     * 如果是通用问题，Hessian以稠密结构存储，除非用户设定某些vertex为边缘
     */
    enum class ProblemType  // 强类型枚举
    {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };
    
    Problem(ProblemType problemType);
    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);
    bool AddEdge(std::shared_ptr<Edge> edge);
    bool RemoveEdge(std::shared_ptr<Edge> edge);
    
    bool Solve(int iterations);

    // (key, value)
    typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
    typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

private:
    // 设置排序后各顶点的id
    void SetOrdering();

    // 构造Hessian矩阵
    void MakeHessian();

    // LM算法初始化，选取初始阻尼因子lambda
    void ComputeLambdaInitLM();

    // 增量方程中添加/减去阻尼因子
    void AddLambdaHessianLM();

    void RemoveLambdaHessianLM();

    // 求解线性方程组
    void SolveLinearSystem();

    // 更新变量
    void UpdateStates();

    // 误差没有减小，回滚
    void RollbackStates();

    // 判断当前更新的可靠性以及阻尼因子lambda的更新
    bool IsGoodStepInLM();

    ProblemType problemType_;
    // vertices_是map类型，顺序是按照id号排序的
    HashVertex vertices_;
    HashEdge edges_;
    HashVertexIdToEdge vertexToEdge_;

    unsigned long ordering_poses_ = 0;
    unsigned long ordering_landmarks_ = 0;
    unsigned long ordering_generic_ = 0;
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;  // 以ordering排序的pose顶点
    std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;  // 以ordering排序的landmark顶点

    // 增量方程参数
    MatXX H_;
    VecX b_;
    VecX delta_x_;
    MatXX H_without_lambda_;

    double time_hessian_ = 0.0;
    double time_solve_ = 0.0;

    double currentLambda_ = -1.0;  // 初始阻尼因子lambda
    double currentChi2_ = 0.0;
    double stopLMThreshold_;  // LM停止迭代的误差阈值
    double ni_ = 2.0;  // Nielsen阻尼更新策略中，控制lambda缩放大小的系数
    
    // 先验信息
    // VecX err_prior_;

};




#endif // !PROBLEM_H