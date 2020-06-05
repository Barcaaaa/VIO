#include "backend/vertex.h"
#include "backend/edge.h"

using namespace std;

unsigned long global_edge_id = 0;

Edge::Edge(int residual_dimension, int num_vertices,
           const std::vector<std::string> &vertices_types)
{
    if (!vertices_types.empty())
        vertices_types_ = vertices_types;
    
    residual_.resize(residual_dimension, 1);
    jacobians_.resize(num_vertices);
    MatXX information(residual_dimension, residual_dimension);
    information.setIdentity();
    information_ = information;
    
    id_ = global_edge_id++;
}

Edge::~Edge()
{

}

double Edge::Chi2()
{
    return residual_.transpose() * information_ * residual_;
}

bool Edge::CheckValid()
{
    if (!vertices_types_.empty())
    {
        for (size_t i = 0; i < vertices_.size(); i++)
        {
            if (vertices_types_[i] != vertices_[i]->TypeInfo())
            {
                cerr << "vertex type does not match, should be " << vertices_types_[i]
                     << ", but set to " << vertices_[i]->TypeInfo() << endl;
                return false;
            }
        }
    }

    return true;
}


