#include <opencv2/imgproc/imgproc.hpp>
#include "Alignment.hpp"
#include "Utils.hpp"
#include "Logger.hpp"

using namespace cv;
using namespace Eigen;


Alignment::Alignment(Matrix3d &leftCameraMatrix, Matrix3d &rightCameraMatrix):
m_leftCameraMatrix(leftCameraMatrix),
m_rightCameraMatrix(rightCameraMatrix)
{

}

std::tuple<Mat, Mat, Matrix3d, Matrix3d> Alignment::alignImages(Matrix3d &rotation, Vector3d &translation, Mat &leftFrame, Mat &rightFrame)
{
    // 1. Find e'
    Vector3d rightEpipole = computeRightEpipole(m_rightCameraMatrix, rotation, translation);

	// 2. Find H' to map e' at infinity
	Matrix3d rightH = findRightHomography(rightEpipole);
    //LOGL(rightH * rightEpipole);

	// 3. Find H
	// 3.1 Find points
	std::vector<std::pair<Vector2d, Vector2d>> points = findPoints(leftFrame, rightFrame, 2000, 35.f);

	// 3.2 Compute H
	Matrix3d leftH = findLeftHomography(rightH, rotation, translation, points);
    Vector3d c(320.,240., 1.);
    c = leftH * c;
    //LOGL(c/c(2));

	// 4. Resample left with H and right image with H'
    Mat Hl = exportGVMatrix(leftH);
    Mat Hr = exportGVMatrix(rightH);
    Mat leftAligned, rightAligned;
	warpPerspective(rightFrame, rightAligned, Hr, Size(640, 480), CV_INTER_CUBIC);
	warpPerspective(leftFrame, leftAligned, Hl, Size(640, 480), CV_INTER_CUBIC);

    return std::make_tuple(leftAligned, rightAligned, leftH, rightH);
}

Matrix3d Alignment::findRightHomography(Vector3d &rightEpipole)
{
	Matrix3d R = computeR(rightEpipole);
	double f = (R * rightEpipole)(0);

	Matrix3d G;
    G << 1./f, 0., 0.,
         0., 1./f, 0.,
		 -1./(f*f), 0., 1./f;

    Matrix3d H = G*R;

	return H;
}

// 1. Rotate epipole in plan xy
// 2. Add scale factor such that z=1
Matrix3d Alignment::computeR(Vector3d &epipole)
{
	// b is normalizedprojection of epipole on plan xz
	Matrix<double, 1, 3> b(epipole(0), 0., epipole(2));
	b /= std::sqrt(b(0) * b(0) + b(2) * b(2));

	// a = normalized epipole
	Vector3d a = epipole;
    a /= a.norm();

	// Compute rotation from a to b
	Vector3d v = a.cross(b);
	double s = v.norm();
	double c = a.dot(b);

    Matrix3d vx = skewSym(v);

	Matrix3d R1 = Matrix3d::Identity() + vx + vx * vx * (1.-c)/(s*s);

	// Compute rotation from R1 * a *lenEpipole to (f, 0, 1)
	double lenEpipole = epipole.norm();
	Vector3d tmp = R1 * a;
	tmp *= lenEpipole;

	double x = tmp(0);
	double z = tmp(2);
	double lenTmp = std::sqrt(x*x + z*z); //y = 0
	double x2 = std::sqrt(lenTmp * lenTmp - 1);
	if(x < 0.)
		x2 = -x2;
	double angle1 = atan2(z, x);
	double angle2 = atan2(1., x2); 
	while(angle1 < 0.)
		angle1 += 2 * M_PI;
	while(angle2 < 0.)
		angle2 += 2 * M_PI;
	double theta = angle1 - angle2;
    Matrix3d ry;
    ry = AngleAxisd(theta, Vector3d::UnitY());

	Matrix3d R = ry * R1;

	return R;
}

Matrix3d Alignment::findLeftHomography(Matrix3d &rightHomography, Matrix3d &rotation, Vector3d &translation, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
	Matrix3d M = computeM(rotation, translation);
	Matrix3d H0 = rightHomography * M;

    unsigned int n = points.size();
	MatrixX3d A;
    VectorXd b;
    A.resize(n, 3);
    b.resize(n);
	for(unsigned int i = 0; i < n; ++i)
	{
		Vector3d xl(points[i].first(0), points[i].first(1), 1.);
		Vector3d xr(points[i].second(0), points[i].second(1), 1.);

		Vector3d xl2 = H0 * xl;
        xl2 /= xl2(2);
		Vector3d xr2 = rightHomography * xr;
        xr2 /= xr2(2);
		A.row(i) = xl2.transpose();
		b.row(i) = Matrix<double, 1, 1>::Ones() * xr2(0);
	}

    JacobiSVD<MatrixX3d> svd(A, ComputeFullU | ComputeFullV);
	Vector3d x = svd.solve(b);
    
	Matrix3d Ha;
    Ha << x(0), x(1), x(2),
		    0., 1., 0.,
			0., 0., 1.;

	return Ha * H0;
}

// tmp = K'P' = [M|m]
Matrix3d Alignment::computeM(Matrix3d &rotation, Vector3d &translation)
{
	Matrix<double, 3, 4> rightP = computeRightProjectionMatrix(rotation, translation);
	return m_rightCameraMatrix * rightP.block(0,0,3,3) * m_leftCameraMatrix.inverse();
}

