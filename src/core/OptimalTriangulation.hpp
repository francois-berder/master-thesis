#ifndef __OPTIMALTRIANGULATION_HPP__
#define __OPTIMALTRIANGULATION_HPP__

#include "Triangulation.hpp"

class OptimalTriangulation : public Triangulation
{
    public :

        OptimalTriangulation(Eigen::Matrix3d &leftCameraMatrix, Eigen::Matrix3d &rightCameraMatrix);

        virtual std::vector<Eigen::Vector3d> triangulate(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);

    private :

        Eigen::Matrix3d &m_leftCameraMatrix;
        Eigen::Matrix3d &m_rightCameraMatrix;
};

#endif /* __OPTIMALTRIANGULATION_HPP__ */

