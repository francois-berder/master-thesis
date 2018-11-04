#include <gsl/gsl_poly.h>
#include <lbfgs.h>
#include "OptimalTriangulation.hpp"
#include "Utils.hpp"
#include <iostream>

using namespace Eigen;

namespace
{

// formula 12.5
double evaluateS(double t, double a, double b, double c, double d, double f, double g)
{
	double s1 = (t*t) / (1. + f*f*t*t);
	double ctd2 = c * t + d;
	ctd2 *= ctd2;
	double atb2 = a * t + b;
	atb2 *= atb2;
	double s2 = ctd2 / (atb2 + g*g*ctd2);

	return s1 + s2;
}

// formula 12.7
double evaluateG(double t, double a, double b, double c, double d, double f, double g)
{
    double ctd = c * t + d;
    double atb = a * t + b;

    double tmp = atb*atb + g*g*ctd*ctd;
    tmp *= tmp;
    tmp *= t;

    double tmp3 = (1. + f*f*t*t);
    tmp3 *= tmp3;
    double tmp2 = atb*ctd*tmp3*(a*d-b*c);

    return tmp - tmp2;
}


lbfgsfloatval_t evaluate(
    void *instance,
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t step)
{
    double *data = (double*) instance;
    double a = data[0];
    double b = data[1];
    double c = data[2];
    double d = data[3];
    double f = data[4];
    double f2 = data[5];

    g[0] = evaluateG(x[0], a, b, c, d, f, f2);
    return evaluateS(x[0], a, b, c, d, f, f2);
}

std::vector<double> findRealRoots(double a, double b, double c, double d, double f, double g)
{
    double a2 = a*a, a3 = a*a*a, a4 = a*a*a*a;
    double b2 = b*b, b3 = b*b*b, b4 = b*b*b*b;
    double c2 = c*c, c3 = c*c*c, c4 = c*c*c*c;
    double d2 = d*d, d3 = d*d*d, d4 = d*d*d*d;
    double f2 = f*f, f4 = f*f*f*f;
    double g2 = g*g, g4 = g*g*g*g;

    double coeffs[7];

    coeffs[0] =
        -a*b*d2
        +b2*c*d;

    coeffs[1] =
        +d4*g4
        +2.*b2*d2*g2
        -a2*d2
        +b2*c2
        +b4;

    coeffs[2] =
        +4.*c*d3*g4
        +4.*a*b*d2*g2
        +4.*b2*c*d*g2
        -2.*a*b*d2*f2
        +2.*b2*c*d*f2
        -a2*c*d
        +a*b*c2
        +4.*a*b3;


    coeffs[3] =
        +6.*c2*d2*g4
        +2.*a2*d2*g2
        +8.*a*b*c*d*g2
        +2.*b2*c2*g2
        -2.*a2*d2*f2
        +2.*b2*c2*f2
        +6.*a2*b2;

    coeffs[4] =
        +4.*c3*d*g4
        +4.*a2*c*d*g2
        +4.*a*b*c2*g2
        -a*b*d2*f4
        +b2*c*d*f4
        -2.*a2*c*d*f2
        +2.*a*b*c2*f2
        +4.*a3*b;

    coeffs[5] =
        +c4*g4
        +2*a2*c2*g2
        -a2*d2*f4
        +b2*c2*f4
        +a4;

    coeffs[6] =
        -a2*c*d*f4
        +a*b*c2*f4;

    std::vector<double> roots;

    // Find all roots using GSL
    double z[12];
    gsl_poly_complex_workspace *w = gsl_poly_complex_workspace_alloc(7);
    gsl_poly_complex_solve(coeffs, 7, w, z);
    gsl_poly_complex_workspace_free(w);

    double data[6];
    data[0] = a;
    data[1] = b;
    data[2] = c;
    data[3] = d;
    data[4] = f;
    data[5] = g;

    for(int i = 0; i < 6; ++i)
    {
        // Discard complex roots
        if(fabs(z[2*i+1]) > 1e-6)
            continue;

        // For each real roots, get better approximation using L-BFGS
        lbfgsfloatval_t *x = lbfgs_malloc(1);
        x[0] = z[2*i];

        int ret = lbfgs(1, x, NULL, evaluate, NULL, data, NULL);

        if(ret == LBFGS_SUCCESS || ret == LBFGS_ALREADY_MINIMIZED)
            roots.push_back(x[0]);

        lbfgs_free(x);
    }

    return roots;
}

Matrix3d computeT(Vector2d &p)
{
    Matrix3d T;

    T << 1., 0., -p(0),
         0., 1., -p(1),
         0., 0., 1.;

    return T;
}


Matrix3d computeR(Vector3d &e)
{
	Matrix3d R;

    R << e(0), e(1), 0.,
        -e(1), e(0), 0.,
		   0., 0., 1.;

    return R;
}

// l = (tf, 1, -t)
Vector3d computeLeftL(double t, double f)
{
	return Vector3d(t * f, 1., -t);
}

// l' = (-f'(ct + d), at+b ct+d)
Vector3d computeRightL(double t, double f, double a , double b, double c, double d)
{
	return Vector3d(-f * (c * t + d), a * t + b, c * t + d);
}

// l = (a, b, c)
// p = (-ac, -bc, a*a + b*b)
Vector3d computeClosestPoint(Vector3d &l)
{
    double a = l(0);
    double b = l(1);
    double c = l(2);

	return Vector3d(-a*c, -b*c, a*a + b*b);
}

}

OptimalTriangulation::OptimalTriangulation(Eigen::Matrix3d &leftCameraMatrix, Eigen::Matrix3d &rightCameraMatrix):
Triangulation(),
m_leftCameraMatrix(leftCameraMatrix),
m_rightCameraMatrix(rightCameraMatrix)
{

}

std::vector<Vector3d> OptimalTriangulation::triangulate(Matrix3d &rotation, Vector3d &translation, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
    Matrix<double, 3, 4> leftProjectionMatrix = computeLeftProjectionMatrix();
    Matrix<double, 3, 4> rightProjectionMatrix = computeRightProjectionMatrix(rotation, translation);

    Matrix3d F = m_rightCameraMatrix.inverse().transpose() * skewSym(translation) * rotation * m_leftCameraMatrix.inverse();

    std::vector<Vector3d> points3D;
    for(auto &e : points)
    {
        Vector3d tmpL = m_leftCameraMatrix * Vector3d(e.first(0), e.first(1), 1.);
        Vector3d tmpR = m_rightCameraMatrix * Vector3d(e.second(0), e.second(1), 1.);
        Vector2d lp(tmpL(0)/tmpL(2), tmpL(1)/tmpL(2));
        Vector2d rp(tmpR(0)/tmpR(2), tmpR(1)/tmpR(2));

		// (i)
		Matrix3d leftT = computeT(lp);
		Matrix3d rightT = computeT(rp);

		// (ii)
		Matrix3d FII = rightT.inverse().transpose() * F * leftT.inverse();

        Matrix3d rotationII;
        Vector3d translationII;
        std::tie(rotationII, translationII) = extractRotationTranslation(FII, m_leftCameraMatrix, m_rightCameraMatrix, points);

        Vector3d leftEpipole = computeLeftEpipole(m_leftCameraMatrix, translationII);
        Vector3d rightEpipole = computeRightEpipole(m_rightCameraMatrix, rotationII, translationII);

		double lambda = 1. / std::sqrt(leftEpipole(0) * leftEpipole(0) + leftEpipole(1) * leftEpipole(1));
		leftEpipole *= lambda;
		lambda = 1. / std::sqrt(rightEpipole(0) * rightEpipole(0) + rightEpipole(1) * rightEpipole(1));
		rightEpipole *= lambda;

		// (iv)
		Matrix3d leftR = computeR(leftEpipole);
		Matrix3d rightR = computeR(rightEpipole);

		// (v)
		Matrix3d FV = rightR * FII * leftR.transpose();

		// (vi)
		double f = leftEpipole(2);
		double g = rightEpipole(2);
		double a = FV(1, 1);
		double b = FV(1, 2);
		double c = FV(2, 1);
		double d = FV(2, 2);

		// (vii)
		std::vector<double> ts = findRealRoots(a, b, c, d, f, g);

		// (viii)
		double asymptoticT = (1./(f*f)) + (c*c)/(a*a + g*g*c*c);
		ts.push_back(asymptoticT);

		std::vector<double> costs;
		for(unsigned int j = 0; j < ts.size(); ++j)
			costs.push_back(evaluateS(ts[j], a, b, c, d, f, g));
        int index = std::min_element(costs.begin(), costs.end()) - costs.begin();
		double minT = ts[index];

		// (ix)
		Vector3d leftL = computeLeftL(minT, f);
		Vector3d rightL = computeRightL(minT, g, a , b, c, d);
	    Vector3d xl = computeClosestPoint(leftL);
        Vector3d xr = computeClosestPoint(rightL);

		// (x)
		xl = m_leftCameraMatrix.inverse() * leftT.inverse() * leftR.transpose() * xl;
		xr = m_rightCameraMatrix.inverse() * rightT.inverse() * rightR.transpose() * xr;
        xl /= xl(2);
        xr /= xr(2);

		// (xi) Homogeneous solution
        Matrix4d A;
        A.row(0) = leftProjectionMatrix.row(2).transpose() * xl(0) - leftProjectionMatrix.row(0).transpose();
        A.row(1) = leftProjectionMatrix.row(2).transpose() * xl(1) - leftProjectionMatrix.row(1).transpose();
        A.row(2) = rightProjectionMatrix.row(2).transpose() * xr(0) - rightProjectionMatrix.row(0).transpose();
        A.row(3) = rightProjectionMatrix.row(2).transpose() * xr(1) - rightProjectionMatrix.row(1).transpose();

        JacobiSVD<Matrix4d> svd(A, ComputeFullV);
        Vector3d point = svd.matrixV().col(3).block(0,0,3,1) / svd.matrixV()(3,3);
        points3D.push_back(point);
    }

    return points3D;
}

