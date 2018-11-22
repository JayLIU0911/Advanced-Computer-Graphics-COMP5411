#include "glcamera.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLCamera::GLCamera(const Eigen::Vector3f& sceneCenter, float sceneRadius, float aspectRatio, float fovY) {
	mEPSILON = 1e-7;

	mModelViewStale = true;
	mProjStale = true;

	mWorldUp = Eigen::Vector3f(0, 1, 0);

	setAspectRatio(aspectRatio);
	setFieldOfView(fovY);

	mForward = Eigen::Vector3f(0, 0, -1); // May differ
	mUp = mWorldUp; // May differ

	if (isParallel(mUp, mForward)) {
		if (isParallel(mForward, Eigen::Vector3f(0, 1, 0))) {
			mForward = Eigen::Vector3f(0, 0, -1);
		}
		mUp = mWorldUp;
	}

	// Ensure the basis vectors are orthogonal and normalized
	mForward.normalize();
	mRight = (mForward.cross(mUp)).normalized();
	mUp = (mRight.cross(mForward)).normalized();

	Eigen::Vector3f pos = sceneCenter - sceneRadius / std::sin(getFieldOfView() * M_PI / 360.0) * mForward;
	setPosition(pos);

	float nearPlane = computeZNear(sceneCenter, getPosition());
	float farPlane = computeZFar(sceneCenter, getPosition());
	setNearPlane(nearPlane);
	setFarPlane(farPlane);

	recomputeModelViewMat();
	recomputeProjMat();

}

void GLCamera::moveForward(float amount) {
	// Moves the camera along its forward vector
	mPosition += amount * mForward;
	mModelViewStale = true;
}

void GLCamera::moveBackward(float amount) {
	// Moves the camera along its backward vector
	moveForward(-amount);
}

void GLCamera::moveRight(float amount) {
	// Moves the camera along its right vector
	mPosition += amount * mRight;
	mModelViewStale = true;
}

void GLCamera::moveLeft(float amount) {
	// Moves the camera along its left vector
	moveRight(-amount);
}

void GLCamera::moveUp(float amount) {
	// Moves the camera along its up vector
	mPosition += amount * mUp;
	mModelViewStale = true;
}

void GLCamera::moveDown(float amount) {
	// Moves the camera along its down vector
	moveUp(-amount);
}

void GLCamera::lookAt(const Eigen::Vector3f& pos) {
	// Turns the camera to face a world space position
	if (isSamePoint(pos, mPosition)) {
		return;
	}
	mForward = (pos - mPosition).normalized();
	if (isParallel(mForward, mWorldUp)) {
		if (isParallel(mRight, mForward)) {
			mRight = (mForward.cross(mUp)).normalized();
			mUp = (mRight.cross(mForward)).normalized();
		} else {
			Eigen::Vector3f upTemp = mRight.cross(mForward);
			mUp = upTemp.normalized() * (mUp.dot(upTemp) > 0 ? 1.0 : -1.0);
			mRight = (mForward.cross(mUp)).normalized();
		}
	} else {
		Eigen::Vector3f rightTemp = mForward.cross(mWorldUp);
		mRight = rightTemp.normalized() * (mRight.dot(rightTemp) > 0 ? 1.0 : -1.0);
		mUp = (mRight.cross(mForward)).normalized();
	}
	mModelViewStale = true;
}

void GLCamera::lookAt(const Eigen::Vector3f& pos, const Eigen::Vector3f& up) {
	// Look at position with up
	if (!isSamePoint(pos, mPosition)) {
		mForward = (pos - mPosition).normalized();
	}

	if (isParallel(mForward, up)) {
		if (isParallel(mForward, mRight)) {
			mRight = (mUp.cross(mForward)).normalized();
			mUp = (mRight.cross(mForward)).normalized();
		} else {
			mUp = (mRight.cross(mForward)).normalized();
			mRight = (mForward.cross(mUp)).normalized();
		}
	} else {
		mRight = (mForward.cross(up)).normalized();
		mUp = (mRight.cross(mForward)).normalized();
	}
	mModelViewStale = true;
}

void GLCamera::lookUp(float degrees) {
	// Tilts the camera to look up with FPS-like movement
	mForward = applyTransform(rotateAxis(mRight, degrees), mForward);
	mUp = (mRight.cross(mForward)).normalized();

	mRight = (mForward.cross(mUp)).normalized();

	mModelViewStale = true;
}

void GLCamera::lookDown(float degrees) {
	// Tilts the camera to look down with FPS-like movement
	lookUp(-degrees);
}

void GLCamera::lookRight(float degrees) {
	// Tilts the camera to look right with FPS-like movement
	Eigen::Matrix4f rotate = rotateUp(-degrees);
	mForward = applyTransform(rotate, mForward);
	mUp = applyTransform(rotate, mUp);
	mRight = (mForward.cross(mUp)).normalized();
	mUp = (mRight.cross(mForward)).normalized();
	mModelViewStale = true;
}

void GLCamera::lookLeft(float degrees) {
	// Tilts the camera to look left with FPS-like movement
	lookRight(-degrees);
}

void GLCamera::pitchUp(float degrees) {
	// Pitches the camera up with space flight like movement
	mForward = applyTransform(rotateAxis(mRight, degrees), mForward);
	mUp = (mRight.cross(mForward)).normalized();
	mRight = (mForward.cross(mUp)).normalized();
	mModelViewStale = true;
}

void GLCamera::pitchDown(float degrees) {
	// Pitches the camera down with space flight like movement
	pitchUp(-degrees);
}

void GLCamera::headRight(float degrees) {
	// Heads the camera right with space flight like movement
	mForward = applyTransform(rotateAxis(mUp, -degrees), mForward);
	mRight = (mForward.cross(mUp)).normalized();
	mUp = (mRight.cross(mForward)).normalized();
	mModelViewStale = true;
}

void GLCamera::headLeft(float degrees) {
	// Heads the camera left with space flight like movement
	headRight(-degrees);
}

void GLCamera::rollCw(float degrees) {
	// Rolls the camera clockwise
	rollCcw(-degrees);
}

void GLCamera::rollCcw(float degrees) {
	// Rolls the camera counter-clockwise
	mRight = applyTransform(rotateAxis(mForward, degrees), mRight);
	mUp = (mRight.cross(mForward)).normalized();
	mForward = (mUp.cross(mRight)).normalized();
	mModelViewStale = true;
}

void GLCamera::orbitUp(float length, float degrees) {
	// Moves the camera upwards in orbit around a point
	moveForward(length);
	pitchDown(degrees);
	moveBackward(length);
}

void GLCamera::orbitDown(float length, float degrees) {
	// Moves the camera downwards in orbit around a point
	orbitUp(length, -degrees);
}

void GLCamera::orbitRight(float length, float degrees) {
	// Moves the camera right in orbit around a point
	moveForward(length);
	lookLeft(degrees);
	moveBackward(length);
}

void GLCamera::orbitLeft(float length, float degrees) {
	// Moves the camera left in orbit around a point
	orbitRight(length, -degrees);
}

Eigen::Vector3f GLCamera::getForward() const {
	// Gets a unit vector in the direction the camera is facing
	return mForward;
}

Eigen::Vector3f GLCamera::getRight() const {
	// Gets a unit vector in the direction of the camera's right
	return mRight;
}

Eigen::Vector3f GLCamera::getUp() const {
	// Gets a unit vector in the direction of the camera's up
	return mUp;
}

Eigen::Matrix4f GLCamera::getModelViewMat() {
	// Get modelview matrix
	if (mModelViewStale) {
		recomputeModelViewMat();
	}
	return mModelViewMat;
}

Eigen::Matrix4f GLCamera::getProjectionMat() {
	if (mProjStale) {
		recomputeProjMat();
	}
	return mProjectionMat;
}

Eigen::Matrix4f GLCamera::getTransformMat() {
	return getProjectionMat() * getModelViewMat();
}

float GLCamera::getAspectRatio() const {
	return mAspectRatio;
}

void GLCamera::setAspectRatio(float ar) {
	mAspectRatio = ar;
	mProjStale = true;
}

float GLCamera::getFieldOfView() const {
	return mFovY;
}

void GLCamera::setFieldOfView(float fv) {
	mFovY = fv;
	mProjStale = true;
}

Eigen::Vector3f GLCamera::getPosition() const {
	// Gets the current world space position of the camera
	return mPosition;
}

void GLCamera::setPosition(const Eigen::Vector3f& p) {
	// Sets the current world space position of the camera
	mPosition = p;
	mModelViewStale = true;
}

float GLCamera::getNearPlane() const {
	return mNearPlane;
}

void GLCamera::setNearPlane(float np) {
	mNearPlane = np;
	mProjStale = true;
}

float GLCamera::getFarPlane() const {
	return mFarPlane;
}

void GLCamera::setFarPlane(float np) {
	mFarPlane = np;
	mProjStale = true;
}

bool GLCamera::isZeroVector(const Eigen::Vector3f& v) const {
	return v.dot(v) < mEPSILON;
}

bool GLCamera::isParallel(const Eigen::Vector3f& u, const Eigen::Vector3f& v) const {
	float cosine = (u.normalized()).dot(v.normalized());
	return std::abs(cosine - 1) < mEPSILON || std::abs(cosine + 1) < mEPSILON;
}

bool GLCamera::isSamePoint(const Eigen::Vector3f& p, const Eigen::Vector3f& q) const {
	Eigen::Vector3f v = q - p;
	return isZeroVector(v);
}

Eigen::Matrix4f GLCamera::rotateAxis(const Eigen::Vector3f& axis, float theta) const {
	// Rotation matrix for arbitrary axis.
	Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();

	float radians = theta * M_PI / 180.0;
	float cosine = std::cos(radians);
	float cosineComp = 1 - cosine;
	float sine = std::sin(radians);

	ret(0, 0) = axis[0] * axis[0] + (1 - axis[0] * axis[0]) * cosine;
	ret(0, 1) = cosineComp * axis[0] * axis[1] - axis[2] * sine;
	ret(0, 2) = cosineComp * axis[0] * axis[2] + axis[1] * sine;
	ret(1, 0) = cosineComp * axis[0] * axis[1] + axis[2] * sine;
	ret(1, 1) = axis[1] * axis[1] + (1 - axis[1] * axis[1]) * cosine;
	ret(1, 2) = cosineComp * axis[1] * axis[2] - axis[0] * sine;
	ret(2, 0) = cosineComp * axis[0] * axis[2] - axis[1] * sine;
	ret(2, 1) = cosineComp * axis[1] * axis[2] + axis[0] * sine;
	ret(2, 2) = axis[2] * axis[2] + (1 - axis[2] * axis[2]) * cosine;

	return ret;
}

Eigen::Matrix4f GLCamera::rotateUp(float degrees) const {
	float angle = degrees * M_PI / 180.0;
	Eigen::Matrix4f c = Eigen::Matrix4f::Identity();
	c(0, 0) = std::cos(angle);
	c(2, 2) = c(0, 0);
	c(0, 2) = std::sin(angle);
	c(2, 0) = -c(0, 2);
	return c;
}

Eigen::Vector3f GLCamera::applyTransform(const Eigen::Matrix4f& mat, const Eigen::Vector3f& v) const {
	Eigen::Vector4f vTemp = mat * Eigen::Vector4f(v(0), v(1), v(2), 0.0);

	return Eigen::Vector3f(vTemp(0), vTemp(1), vTemp(2));
}

float GLCamera::computeZNear(const Eigen::Vector3f& sceneCenter, const Eigen::Vector3f& camPos, float clipCoeff) const {
	float distToSceneCenter = (camPos - sceneCenter).norm();
	return distToSceneCenter * clipCoeff;
}

float GLCamera::computeZFar(const Eigen::Vector3f& sceneCenter, const Eigen::Vector3f& camPos, float clipCoeff) const {
	float distToSceneCenter = (camPos - sceneCenter).norm();
	return distToSceneCenter * (1.0 + clipCoeff);
}

void GLCamera::recomputeModelViewMat() {
	Eigen::Matrix4f invRotate = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	for (int c = 0; c < 3; ++c) {
		invRotate(0, c) = mRight(c);
		invRotate(1, c) = mUp(c);
		invRotate(2, c) = -mForward(c);

		translate(c, 3) = -mPosition(c);
	}

	mModelViewMat = invRotate * translate;
	mModelViewStale = false;
}


void GLCamera::recomputeProjMat() {
	float zNear = getNearPlane();
	float zFar = getFarPlane();

	float top = std::tan(getFieldOfView() * M_PI / 360.0) * zNear;
	float right = top * getAspectRatio();

	Eigen::Matrix4f c = Eigen::Matrix4f::Zero();
	c(0, 0) = zNear / right;
	c(1, 1) = zNear / top;
	c(2, 2) = -(zFar + zNear) / (zFar - zNear);
	c(2, 3) = -2.0 * zFar * zNear / (zFar - zNear);
	c(3, 2) = -1.0;

	mProjectionMat = c;
	mProjStale = false;
}
