#ifndef __MIDPOINTTRIANGULATION_HPP__
#define __MIDPOINTTRIANGULATION_HPP__

#include "Triangulation.hpp"

class MidPointTriangulation : public Triangulation
{
    public :

        virtual std::vector<Eigen::Vector3d> triangulate(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);

};

#endif /* __MIDPOINTTRIANGULATION_HPP__ */

