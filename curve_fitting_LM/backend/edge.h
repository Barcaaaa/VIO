#ifndef EDGE_H
#define EDGE_H

#include <iostream>
#include <string>
#include <memory>  // 否则无法找到 shared_ptr

#include "backend/eigen_types.h"

/**
 * 构造边类，边负责计算残差，残差 = 预测值 - 观测值，其维度在构造函数中定义
 * cost function = 残差*信息矩阵*残差，为一个具体数值
 */
class Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    /**
     * @param residual_dimension 残差维度
     * @param num_vertices 顶点数量
     * @param vertices_types 顶点类型名称，可以不给，不给的话check中不会检查
     */
    explicit Edge(int residual_dimension, int num_vertices,
                  const std::vector<std::string> &vertices_types = std::vector<std::string>());  // explicit关键字的作用就是防止类构造函数的隐式自动转换
    virtual ~Edge();

    // 边的id
    unsigned long Id() const { return id_; }

    // 添加顶点
    bool AddVertex(std::shared_ptr<Vertex> vertex)
    {
        vertices_.emplace_back(vertex);
        return true;
    }

    // 设置顶点，按引用顺序排列
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices)
    {
        vertices_ = vertices;
        return true;
    }

    // 返回第i个顶点
    std::shared_ptr<Vertex> GetVertex(int i)
    {
        return vertices_[i];
    }

    // 返回所有顶点
    std::vector<std::shared_ptr<Vertex>> GetVertices() const
    {
        return vertices_;
    }

    // 返回关联顶点的个数
    size_t NumVertices() const { return vertices_.size(); }

    // 返回边的名称，在子类中实现
    virtual std::string TypeInfo() const = 0;

    // 计算残差，在子类中实现
    virtual void ComputeResidual() = 0;

    // 计算雅克比，在子类中实现
    virtual void ComputeJacobians() = 0;

    // 计算平方误差，残差的转置 * 信息矩阵 * 残差
    double Chi2();

    // 返回残差值
    VecX GetResidual() const { return residual_; }

    // 返回雅克比
    std::vector<MatXX> GetJacobians() const { return jacobians_; }

    // 返回信息矩阵
    MatXX GetInformation() const { return information_; }

    // 设置观测信息
    // void SetObservation(const VecX &observation)
    // {
    //     observation_ = observation;
    // }

    // 返回观测信息
    // VecX GetObservation() const { return observation_; }

    // 检查边的信息是否全部设置，debug用
    bool CheckValid();
    
    // 设置排序后的id
    void SetOrderingId(int id)
    {
        ordering_id_ = id;
    }
    
    // 返回排序后的id
    int GetOrderingId() const { return ordering_id_; }

protected:
    unsigned long id_;  // 边的id
    // 在优化问题problem类中经过排序后的id，用于寻找Jacobian对应块
    // ordering_id带有维度信息，例如：ordering_id则对应Hessian中的第6列
    int ordering_id_;

    std::vector<std::string> vertices_types_;  // 各顶点类型信息，用于debug
    std::vector<std::shared_ptr<Vertex>> vertices_;  // 存储该边对应的顶点
    VecX residual_;  // 残差
    std::vector<MatXX> jacobians_;  // 雅克比
    MatXX information_;  // 信息矩阵
    // VecX observation_;  // 观测信息
};


#endif // !EDGE_H