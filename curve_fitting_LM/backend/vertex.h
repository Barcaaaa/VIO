#ifndef VERTEX_H
#define VERTEX_H

#include "backend/eigen_types.h"

/**
 * 构造顶点类，对应一个参数块，变量值以VecX存储，其维度在构造函数中定义
 */
class Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @param num_dimension 顶点自身维度
     * @param local_dimension 局部参数化维度，-1表示与自身维度一样
     */
    explicit Vertex(int num_dimension, int local_dimension = -1);  // explicit关键字的作用就是防止类构造函数的隐式自动转换.
    virtual ~Vertex();

    // 返回变量维度
    int Dimension() const;

    // 返回变量局部维度
    int LocalDimension() const;

    // 顶点的id
    unsigned long Id() const { return id_; }
    
    // 返回参数值
    VecX GetParameters() const { return parameters_; }

    // 返回参数值的引用
    VecX &GetParameters() { return parameters_; }

    // 设置参数值
    void SetParameters(const VecX &params) { parameters_ = params; }

    // 这里默认向量加法，可重定义
    virtual void Plus(const VecX &delta);

    // 返回顶点的名称，在子类中实现，用于debug
    virtual std::string TypeInfo() const = 0;

    // 排序后顶点的id
    int OrderingId() const { return ordering_id_; }

    // 设置排序后的id
    void SetOrderingId(unsigned long id) { ordering_id_ = id; }

    // 固定该点的估计值
    void SetFixed(bool fixed = true) { fixed_ = fixed; }

    // 测试该点是否被固定
    bool IsFixed() const { return fixed_; }

protected:
    unsigned long id_;  // 顶点id，自动生成
    // 在优化问题problem类中经过排序后的id，用于寻找Jacobian对应块
    // ordering_id带有维度信息，例如：ordering_id则对应Hessian中的第6列
    unsigned long ordering_id_ = 0;  // 从0开始

    VecX parameters_;  // 存储的变量值
    int local_dimension_;  // 局部参数化维度
    
    bool fixed_ = false;  // 判断点是否固定
};


#endif // !VERTEX_H