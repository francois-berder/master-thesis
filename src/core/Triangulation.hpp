#ifndef __TRIANGULATION_HPP__
#define __TRIANGULATION_HPP__

#include <vector>
#include <Eigen/Dense>

class Triangulation
{
    public :

        virtual ~Triangulation() = default;

        virtual std::vector<Eigen::Vector3d> triangulate(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points) = 0;
};

#endif /* __TRIANGULATION_HPP__ */

