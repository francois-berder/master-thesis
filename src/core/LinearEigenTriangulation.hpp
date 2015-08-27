#ifndef __LINEAREIGENTRIANGULATION_HPP__
#define __LINEAREIGENTRIANGULATION_HPP__

#include "Triangulation.hpp"

class LinearEigenTriangulation : public Triangulation
{
    public :

        virtual std::vector<Eigen::Vector3d> triangulate(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);

};

#endif /* __LINEAREIGENTRIANGULATION_HPP__ */
