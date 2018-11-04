#include "Utils.hpp"
#include "LinearEigenTriangulation.hpp"

using namespace Eigen;
using namespace cv;


Matrix3d importCVMatrix(Mat &m)
{
    Matrix3d m2;
    m2 << m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2),
        m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2),
        m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2);

    return m2;
}

Mat exportGVMatrix(Matrix3d &m)
{
    Matx33d m2(m(0,0),m(0,1),m(0,2),
               m(1,0),m(1,1),m(1,2),
               m(2,0),m(2,1),m(2,2));

    return Mat(m2);
}

Matrix3d skewSym(Vector3d &v)
{
    Matrix3d m;
    m << 0, -v(2), v(1),
         v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return m;
}

Matrix<double, 3, 4> computeLeftProjectionMatrix()
{
    Matrix<double, 3, 4> leftProjectionMatrix = Matrix<double, 3, 4>::Zero();
    leftProjectionMatrix.block(0,0,3,3) = Matrix3d::Identity();
    return leftProjectionMatrix;
}

Matrix<double, 3, 4> computeRightProjectionMatrix(Matrix3d &rotation, Vector3d &translation)
{
    Matrix<double, 3, 4> rightProjectionMatrix = Matrix<double, 3, 4>::Zero();
    rightProjectionMatrix.block(0,0,3,3) = rotation.transpose();
    rightProjectionMatrix.col(3) = - rotation.transpose() * translation;

    return rightProjectionMatrix;
}

Vector3d computeLeftEpipole(Matrix3d &leftCameraMatrix, Vector3d &translation)
{
    return leftCameraMatrix * translation;
}

Vector3d computeRightEpipole(Matrix3d &rightCameraMatrix, Matrix3d &rotation, Vector3d &translation)
{
    return rightCameraMatrix * -rotation.transpose() * translation;
}

std::tuple<Matrix3d, Vector3d> extractRotationTranslation(Matrix3d &F, Matrix3d &leftCameraMatrix, Matrix3d &rightCameraMatrix, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
	Matrix3d E = rightCameraMatrix.transpose() * F * leftCameraMatrix;

    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);

	Matrix3d W;
    W << 0., -1., 0.,
         1.,  0., 0.,
	     0.,  0., 1.;

	Matrix3d R1 = svd.matrixU() * W * svd.matrixV().transpose();
    Matrix3d R2 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();

	Vector3d T1 = svd.matrixU().col(2);
    T1 /= T1.norm();
	Vector3d T2 = -T1;

    LinearEigenTriangulation let;
    std::vector<Vector3d> points3D1 = let.triangulate(R1, T1, points);
    std::vector<Vector3d> points3D2 = let.triangulate(R2, T1, points);
    std::vector<Vector3d> points3D3 = let.triangulate(R1, T2, points);
    std::vector<Vector3d> points3D4 = let.triangulate(R2, T2, points);

    double avgZ1 = 0.;
    for(auto& p : points3D1)
        avgZ1 += p(2);
    avgZ1 /= static_cast<double>(points3D1.size());

    double avgZ2 = 0.;
    for(auto& p : points3D2)
        avgZ2 += p(2);
    avgZ2 /= static_cast<double>(points3D2.size());

    double avgZ3 = 0.;
    for(auto& p : points3D3)
        avgZ3 += p(2);
    avgZ3 /= static_cast<double>(points3D3.size());

    double avgZ4 = 0.;
    for(auto& p : points3D4)
        avgZ1 += p(2);
    avgZ4 /= static_cast<double>(points3D4.size());

    if(avgZ1 > 0.)
        return std::make_tuple(R1, T1);
    else if(avgZ2 > 0.)
        return std::make_tuple(R2, T1);
    else if(avgZ3 > 0.)
        return std::make_tuple(R1, T2);
    else
        return std::make_tuple(R2, T2);
}

double computeReprojectionError(Matrix3d &leftCameraMatrix, Matrix3d &rightCameraMatrix, Matrix3d &rotation, Vector3d &translation, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
    std::vector<std::pair<Vector2d, Vector2d>> pts;
    for(auto& e : points)
    {
        Vector3d xl(e.first(0), e.first(1), 1.);
        Vector3d xr(e.second(0), e.second(1), 1.);
        xl = leftCameraMatrix.inverse() * xl;
        xr = rightCameraMatrix.inverse() * xr;

        Vector2d lp(xl(0), xl(1));
        Vector2d rp(xr(0), xr(1));
        pts.push_back(std::make_pair(lp, rp));
    }

    LinearEigenTriangulation let;
    std::vector<Vector3d> points3D = let.triangulate(rotation, translation, pts);
    Matrix<double, 3, 4> leftProjectionMatrix = computeLeftProjectionMatrix();
    Matrix<double, 3, 4> rightProjectionMatrix = computeRightProjectionMatrix(rotation, translation);

    double error = 0.;
    for(unsigned int i = 0; i < points3D.size(); ++i)
    {
        Vector3d p = points3D[i];
        Vector4d X(p(0), p(1), p(2), 1.);
        Vector3d xl = leftCameraMatrix * leftProjectionMatrix * X;
        Vector3d xr = rightCameraMatrix * rightProjectionMatrix * X;
        xl /= xl(2);
        xr /= xr(2);
        Vector2d lp(xl(0), xl(1));
        Vector2d rp(xr(0), xr(1));

        error += ((lp - points[i].first).norm() + (rp - points[i].second).norm()) / 2.;
    }
    error /= static_cast<double>(points3D.size());

    return error;
}

std::vector<std::pair<Vector2d, Vector2d>> findPoints(Mat &leftFrame, Mat &rightFrame, unsigned int nbFeatures, float distance)
{
	std::vector<KeyPoint> leftKeypoints, rightKeypoints;
	Mat leftDescriptors, rightDescriptors;

	OrbFeatureDetector detector(nbFeatures);
	OrbDescriptorExtractor extractor;
	detector.detect(leftFrame, leftKeypoints);
	detector.detect(rightFrame, rightKeypoints);
	extractor.compute(leftFrame, leftKeypoints, leftDescriptors);
	extractor.compute(rightFrame, rightKeypoints, rightDescriptors);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	std::vector<std::vector<DMatch>> matches;
	matcher->radiusMatch(leftDescriptors, rightDescriptors, matches, distance);

	std::vector<std::pair<Vector2d, Vector2d>> points;
	for(unsigned int i = 0; i < matches.size(); ++i)
	{
		if(matches[i].empty())
			continue;
		DMatch m = matches[i][0];
		Point2f lp = leftKeypoints[m.queryIdx].pt;
		Point2f rp = rightKeypoints[m.trainIdx].pt;
        points.push_back(std::make_pair(Vector2d(lp.x, lp.y), Vector2d(rp.x, rp.y)));
	}

	return points;
}

std::tuple<double, double, double> findCardanAngles(Matrix3d &rotation)
{
    double x = atan2(rotation(2, 1), rotation(2, 2));
    double y = atan2(-rotation(2, 0), std::sqrt(rotation(2, 1)*rotation(2, 1) + rotation(2, 2)*rotation(2, 2)));
    double z = atan2(rotation(1, 0), rotation(0, 0));

    return std::make_tuple(x, y, z);
}

double convertRadToDeg(double angle)
{
    return angle * 180. / M_PI;
}


