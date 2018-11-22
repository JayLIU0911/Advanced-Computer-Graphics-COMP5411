#ifndef UTIL_H
#define UTIL_H
#include <Eigen/Dense>

#ifndef VCOLOR_BLUE
#define VCOLOR_BLUE Eigen::Vector3f(94.0 / 255.0, 185.0 / 255.0, 238.0 / 255.0)
#endif
#ifndef VCOLOR_PURPLE
#define VCOLOR_PURPLE Eigen::Vector3f(0.4f, 0.4f, 0.6f)
#endif

// Some helper functions for computing triangle normals or areas
static Eigen::Vector3f triangleNormal(const Eigen::Vector3f& p1,
                                      const Eigen::Vector3f& p2,
                                      const Eigen::Vector3f& p3) {
	return ((p2 - p1).cross(p3 - p1)).normalized();
}

static double triangleArea(const Eigen::Vector3f& p1,
                           const Eigen::Vector3f& p2,
                           const Eigen::Vector3f& p3) {
	Eigen::Vector3d v1 = Eigen::Vector3d(p2(0), p2(1), p2(2)) - Eigen::Vector3d(p1(0), p1(1), p1(2));
	Eigen::Vector3d v2 = Eigen::Vector3d(p3(0), p3(1), p3(2)) - Eigen::Vector3d(p1(0), p1(1), p1(2));
	return v1.cross(v2).norm() / 2.0;
}

static double triangleCot(const Eigen::Vector3f& p1,
                          const Eigen::Vector3f& p2,
                          const Eigen::Vector3f& p3) {
	// Angle at p2, use double type to deal with precision issues
	Eigen::Vector3d v1 = Eigen::Vector3d(p1(0), p1(1), p1(2)) - Eigen::Vector3d(p2(0), p2(1), p2(2));
	Eigen::Vector3d v2 = Eigen::Vector3d(p3(0), p3(1), p3(2)) - Eigen::Vector3d(p2(0), p2(1), p2(2));

	double _dot_res = v1.normalized().dot(v2.normalized());
	if (_dot_res < -1.0) {
		_dot_res = -1.0;
	} else if (_dot_res > 1.0) {
		_dot_res = 1.0;
	}
	return 1.0 / std::tan(std::acos(_dot_res));
}

static Eigen::Vector3f gluProject(const Eigen::Matrix4f& mvp,
                                  const Eigen::Vector4f& viewport,
                                  const Eigen::Vector3f& pt) {
	Eigen::Vector4f ptHomo(pt(0), pt(1), pt(2), 1.0);
	Eigen::Vector4f ptProj = mvp * ptHomo;
	ptProj /= ptProj(3); // TO NDC

	float winX = viewport(0) + 0.5 * viewport(2) * (ptProj(0) + 1.0);
	float winY = viewport(1) + 0.5 * viewport(3) * (ptProj(1) + 1.0);
	float winZ = 0.5 * (ptProj(2) + 1.0);
	return Eigen::Vector3f(winX, viewport(3) - winY, winZ);
}

static Eigen::Vector3f gluUnproject(const Eigen::Matrix4f& mvpInv,
                                    const Eigen::Vector4f& viewport,
                                    const Eigen::Vector3f& pt) {
	Eigen::Vector4f ndc(2.0 * (pt(0) - viewport(0)) / viewport(2) - 1.0,
	                    2.0 * (viewport(3) - pt(1) - viewport(1)) / viewport(3) - 1.0,
	                    2.0 * pt(2) - 1.0,
	                    1.0);

	Eigen::Vector4f objPt = mvpInv * ndc;
	return Eigen::Vector3f(objPt(0), objPt(1), objPt(2));
}

// Pseudo inverse of a matrix in Eigen
template < typename _Matrix_Type_ >
static _Matrix_Type_ eigenPinv(const _Matrix_Type_& a,
                               double epsilon = std::numeric_limits< double >::epsilon()) {
	if (a.rows() < a.cols()) {
		Eigen::JacobiSVD< _Matrix_Type_ > svd(a.transpose(),
		                                      Eigen::ComputeThinU | Eigen::ComputeThinV);

		double tolerance = epsilon * std::max((double)a.cols(), (double)a.rows()) *
		                   svd.singularValues().array().abs().maxCoeff();

		return (svd.matrixV() *
		        (svd.singularValues().array().abs() > tolerance)
		        .select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
		        svd.matrixU().adjoint()
		).transpose();
	}

	Eigen::JacobiSVD< _Matrix_Type_ > svd(a,
	                                      Eigen::ComputeThinU | Eigen::ComputeThinV);

	double tolerance = epsilon * std::max((double)a.cols(), (double)a.rows()) *
	                   svd.singularValues().array().abs().maxCoeff();

	return svd.matrixV() *
	       (svd.singularValues().array().abs() > tolerance)
	       .select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
	       svd.matrixU().adjoint();
}

#endif // UTIL_H
