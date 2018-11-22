#ifndef GL_CAMERA_H
#define GL_CAMERA_H
#include <Eigen/Dense>

// Represents a camera in world space
// You DO NOT need to understand or use it.
class GLCamera {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	GLCamera(const Eigen::Vector3f& sceneCenter,
	         float sceneRadius,
	         float aspectRatio = 1.0,
	         float fovY = 45.0);

	void moveForward(float amount);
	void moveBackward(float amount);
	void moveRight(float amount);
	void moveLeft(float amount);
	void moveUp(float amount);
	void moveDown(float amount);

	void lookAt(const Eigen::Vector3f& pos);
	void lookAt(const Eigen::Vector3f& pos, const Eigen::Vector3f& up);
	void lookUp(float degrees);
	void lookDown(float degrees);
	void lookRight(float degrees);
	void lookLeft(float degrees);

	void pitchUp(float degrees);
	void pitchDown(float degrees);

	void headRight(float degrees);
	void headLeft(float degrees);

	void rollCw(float degrees);
	void rollCcw(float degrees);

	void orbitUp(float length, float degrees);
	void orbitDown(float length, float degrees);
	void orbitRight(float length, float degrees);
	void orbitLeft(float length, float degrees);

	Eigen::Vector3f getForward() const;
	Eigen::Vector3f getRight() const;
	Eigen::Vector3f getUp() const;

	Eigen::Matrix4f getModelViewMat();
	Eigen::Matrix4f getProjectionMat();
	Eigen::Matrix4f getTransformMat();

	float getAspectRatio() const;
	void setAspectRatio(float ar);

	float getFieldOfView() const;
	void setFieldOfView(float fv);

	Eigen::Vector3f getPosition() const;
	void setPosition(const Eigen::Vector3f& p);

	float getNearPlane() const;
	void setNearPlane(float np);
	float getFarPlane() const;
	void setFarPlane(float np);

private:
	bool isZeroVector(const Eigen::Vector3f& v) const;
	bool isParallel(const Eigen::Vector3f& u, const Eigen::Vector3f& v) const;
	bool isSamePoint(const Eigen::Vector3f& p, const Eigen::Vector3f& q) const;

	Eigen::Matrix4f rotateAxis(const Eigen::Vector3f& axis, float degrees) const;
	Eigen::Matrix4f rotateUp(float degrees) const;
	Eigen::Vector3f applyTransform(const Eigen::Matrix4f& mat, const Eigen::Vector3f& v) const;

	float computeZNear(const Eigen::Vector3f& sceneCenter, const Eigen::Vector3f& camPos, float clipCoeff = 0.1) const;
	float computeZFar(const Eigen::Vector3f& sceneCenter, const Eigen::Vector3f& camPos, float clipCoeff = 10) const;

	void recomputeModelViewMat();
	void recomputeProjMat();

private:
	float mEPSILON;
	bool mModelViewStale;
	bool mProjStale;

	Eigen::Vector3f mWorldUp;
	Eigen::Vector3f mUp;
	Eigen::Vector3f mForward;
	Eigen::Vector3f mRight;
	Eigen::Vector3f mPosition;

	Eigen::Matrix4f mModelViewMat;
	Eigen::Matrix4f mProjectionMat;

	float mAspectRatio;
	float mNearPlane;
	float mFarPlane;
	float mFovY;
};

#endif // GL_CAMERA_H
