#ifndef VIEWER_3D_H
#define VIEWER_3D_H
#include <nanogui/nanogui.h>
#include "glcamera.h"
#include "mesh.h"
#include "deformer.h"

// Data manager for the viewer
// You DO NOT need to understand or use it.
class ViewerData {
public:
	enum class ShadeMode { FlatShade, SmoothShade, None };

	ViewerData();
	~ViewerData();

	// Draw data
	void draw();

	GLCamera* getCamera();
	const Eigen::Vector3f& getSceneCenter() const;
	float getSceneRadius() const;

	// Draw options
	bool useWireframe() const;
	void setUseWireframe(bool b);
	ShadeMode getShadeMode() const;
	void setShadeMode(ShadeMode sm);

	void setWindowSize(int w, int h);
	void setMesh(Mesh* m, float aspectRatio);

private:
	void updateVertexInfo();

	// Member variables
	nanogui::GLShader* mMeshShader;
	nanogui::GLShader* mWireframeShader;

	Mesh* mMesh;
	Eigen::Matrix3Xf mMeshVertices;
	Eigen::Matrix3Xf mMeshPerFaceNormals;
	Eigen::Matrix3Xf mMeshPerVertexNormals;
	Eigen::Matrix3Xf mMeshColors;
	Eigen::Matrix3Xf mWireframeVertices;
	Eigen::MatrixXi mWireframeFlags;

	bool mDrawWireframe;
	ShadeMode mShadeMode;

	GLCamera* mCamera;
	Eigen::Vector3f mSceneCenter;
	float mSceneRadius;
};

// A 3D model viewer
// You DO NOT need to understand or use it.
class Viewer3D {
public:
	// UI Enumerations
	enum class MouseButton { Left, Middle, Right, None };

	enum class SmoothScheme { Explicit, Implicit };

	enum class HandleState { Add, Move };

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Viewer3D();
	~Viewer3D();

	// Start of the viewer
	int launch();

	// Draw everything
	void draw();

	// Callbacks for the keyboard and the mouse buttons 
	bool keyPressed(unsigned int unicode_key, int modifier);
	bool keyDown(int key, int modifier);
	bool keyUp(int key, int modifier);

	bool mouseDown(MouseButton button, int modifier);
	bool mouseUp(MouseButton button, int modifier);
	bool mouseMove(int mouse_x, int mouse_y);
	bool mouseScroll(float delta_y);

	// OpenGL context resize
	void resize(int w, int h); // explicitly set window size
	void postResize(int w, int h); // external resize due to user interaction

	nanogui::Screen* getScreen();

private:
	// Init helper functions
	int launchInit();
	bool launchRendering(bool loop);
	void launchShut();

	void openDialogLoadMesh();
	void clearMesh();

	// Smoothing
	void smoothMesh();
	// Deformation
	void buildDeformMat();
	void clearHandles();
	void selectVertexByRect(const Eigen::Vector2f& bboxMin,
	                        const Eigen::Vector2f& bboxMax);
	int selectHandle(float mouseX, float mouseY);
	void moveHandle(float lastMouseX, float lastMouseY,
	                float currentMouseX, float currentMouseY);

	// UI related
	GLFWwindow* mWindow;
	nanogui::FormHelper* mNGui;
	nanogui::Screen* mNScreen;

	int mMouseCurrentX;
	int mMouseCurrentY;
	int mMouseDownX;
	int mMouseDownY;
	MouseButton mMouseDownButton;

	Eigen::Vector4f mBgColor;
	int mWindowWidth;
	int mWindowHeight;

	// Mesh related
	Mesh* mMesh;
	ViewerData* mViewerData;
	SmoothScheme mSmoothScheme;
	bool mUseCotWeights;
	HandleState mHandleState;
	int mSelectedHandle;
	Deformer* mDeformer;
};

#endif // VIEWER_3D_H
