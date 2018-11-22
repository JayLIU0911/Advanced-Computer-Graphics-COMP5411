#include "viewer3d.h"

#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#endif

#include <chrono>
#include <thread>

#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   define __gl_h_ /* Prevent inclusion of the old gl.h */
#else
#   include <GL/gl.h>
#endif

//#define GLFW_INCLUDE_GLU
#if defined(__APPLE__)
#define GLFW_INCLUDE_GLCOREARB
#else
#define GL_GLEXT_PROTOTYPES
#endif

#include <iostream>
#include <igl/file_dialog_open.h>

// -- Implementation of ViewerData --
ViewerData::ViewerData() : mMeshShader(nullptr),
                           mWireframeShader(nullptr),
                           mMesh(nullptr),
                           mCamera(nullptr),
                           mDrawWireframe(false),
                           mShadeMode(ShadeMode::FlatShade) {
	// Shader sources
	std::string meshVertShader = "#version 330\n"
			"in vec3 vin_position;"
			"in vec3 vin_normal;"
			"in vec3 vin_color;"
			"out vec3 vout_mvPosition;"
			"out vec3 vout_mvNormal;"
			"out vec3 vout_color;"
			"uniform mat4 uni_modelview;"
			"uniform mat4 uni_project;"
			"void main() {"
			"    vout_mvPosition = vec3(uni_modelview * vec4(vin_position, 1.0));"
			"    vout_mvNormal = mat3(transpose(inverse(uni_modelview))) * vin_normal;"
			"    gl_Position = uni_project * vec4(vout_mvPosition, 1.0);"
			"    vout_color = vin_color;"
			"}";
	std::string meshFragShader = "#version 330\n"
			"in vec3 vout_mvPosition;"
			"in vec3 vout_mvNormal;"
			"in vec3 vout_color;"
			"out vec4 fout_color;"
			"uniform mat4 uni_modelview;"
			"uniform vec3 uni_lightPosition;"
			"uniform vec3 uni_eyePosition;"
			"void main() {"
			"    vec3 mvLightPosition = vec3(uni_modelview * vec4(uni_lightPosition, 1.0));"
			"    vec3 mvEyePosition = vec3(uni_modelview * vec4(uni_eyePosition, 1.0));"
			"    vec3 lightColor = vec3(1.0, 1.0, 1.0);"
			"    vec3 ambient = 0.3 * lightColor;"
			"    vec3 vertNormal = normalize(vout_mvNormal);"
			"    vec3 lightDir = normalize(mvLightPosition - vout_mvPosition);"
			"    float diff = max(dot(vertNormal, lightDir), 0.0);"
			"    vec3 diffuse = diff * lightColor;"
			"    vec3 viewDir = normalize(mvEyePosition - vout_mvPosition);"
			"    vec3 reflectDir = reflect(-lightDir, vertNormal);"
			"    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);"
			"    vec3 specular = 0.5 * spec * lightColor;"
			"    vec3 combinedColor = (ambient + diffuse + specular) * vout_color;"
			"    fout_color = vec4(min(combinedColor.x, 1.0), min(combinedColor.y, 1.0), min(combinedColor.z, 1.0), 1.0);"
			"}";

	std::string wireframeVertShader = "#version 330\n"
			"in vec3 vin_position;"
			"in int vin_flag;"
			"out vec3 vout_color;"
			"uniform mat4 uni_modelview;"
			"uniform mat4 uni_project;"
			"void main() {"
			"    gl_Position = uni_project * uni_modelview * vec4(vin_position, 1.0);"
			"    if (vin_flag > 0) {"
			"        vout_color = vec3(1.0, 0.0, 0.0);"
			"    } else {"
			"        vout_color = vec3(0.0, 0.0, 0.0);"
			"    }"
			"}";
	std::string wireframeFragShader = "#version 330\n"
			"in vec3 vout_color;"
			"out vec4 fout_color;"
			"void main() {"
			"    fout_color = vec4(vout_color, 1.0);"
			"}";

	mMeshShader = new nanogui::GLShader();
	mMeshShader->init("mesh_shader", meshVertShader, meshFragShader);
	mWireframeShader = new nanogui::GLShader();
	mWireframeShader->init("wireframe_shader", wireframeVertShader, wireframeFragShader);
}

ViewerData::~ViewerData() {
	if (mMeshShader) {
		mMeshShader->free();
		delete mMeshShader;
	}
	mMeshShader = nullptr;
	if (mWireframeShader) {
		mWireframeShader->free();
		delete mWireframeShader;
	}
	mWireframeShader = nullptr;
	if (mCamera) {
		delete mCamera;
	}
	mCamera = nullptr;
}

void ViewerData::draw() {
	if (mMesh == nullptr || mCamera == nullptr) {
		return;
	}

	glEnable(GL_DEPTH_TEST);
	if (mDrawWireframe) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
	}
	updateVertexInfo();

	Eigen::Matrix4f projMat = mCamera->getProjectionMat();
	Eigen::Matrix4f mvMat = mCamera->getModelViewMat();
	Eigen::Vector3f camPos = mCamera->getPosition();

	if (mShadeMode != ShadeMode::None) {
		mMeshShader->bind();
		mMeshShader->setUniform("uni_modelview", mvMat);
		mMeshShader->setUniform("uni_project", projMat);
		mMeshShader->setUniform("uni_lightPosition", camPos);
		mMeshShader->setUniform("uni_eyePosition", camPos);

		mMeshShader->drawArray(GL_TRIANGLES, 0, 3 * mMesh->faces().size());
	}
	if (mDrawWireframe) {
		mWireframeShader->bind();
		mWireframeShader->setUniform("uni_modelview", mvMat);
		mWireframeShader->setUniform("uni_project", projMat);
		mMeshShader->drawArray(GL_LINES, 0, 2 * (mMesh->edges().size() + mMesh->boundaryEdges().size()));

		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	glDisable(GL_DEPTH_TEST);
}


GLCamera* ViewerData::getCamera() {
	return mCamera;
}

const Eigen::Vector3f& ViewerData::getSceneCenter() const {
	return mSceneCenter;
}

float ViewerData::getSceneRadius() const {
	return mSceneRadius;
}

bool ViewerData::useWireframe() const {
	return mDrawWireframe;
}

void ViewerData::setUseWireframe(bool b) {
	mDrawWireframe = b;
}

ViewerData::ShadeMode ViewerData::getShadeMode() const {
	return mShadeMode;
}

void ViewerData::setShadeMode(ShadeMode sm) {
	mShadeMode = sm;
	if (mShadeMode == ShadeMode::FlatShade) {
		mMeshShader->bind();
		mMeshShader->uploadAttrib("vin_normal", mMeshPerFaceNormals);
	} else if (mShadeMode == ShadeMode::SmoothShade) {
		mMeshShader->bind();
		mMeshShader->uploadAttrib("vin_normal", mMeshPerVertexNormals);
	}
}

void ViewerData::setWindowSize(int w, int h) {
	if (mCamera) {
		mCamera->setAspectRatio(float(w) / float(h));
	}
}

void ViewerData::setMesh(Mesh* m, float aspectRatio) {
	mMesh = m;
	if (mMesh == nullptr) {
		return;
	}

	Eigen::Vector3f bboxMin = mMesh->initBboxMin();
	Eigen::Vector3f bboxMax = mMesh->initBboxMax();
	mSceneCenter = (bboxMin + bboxMax) / 2.0;
	mSceneRadius = (bboxMax - bboxMin).norm() / 2.0;
	if (mCamera) {
		delete mCamera;
	}
	mCamera = new GLCamera(mSceneCenter, mSceneRadius, aspectRatio);

	int numHEdges = mMesh->edges().size();
	int numBHEdges = mMesh->boundaryEdges().size();
	mWireframeFlags = Eigen::MatrixXi::Zero(1, 2 * (numHEdges + numBHEdges));
	mWireframeFlags.rightCols(2 * numBHEdges) = Eigen::MatrixXi::Zero(1, 2 * numBHEdges);
	mWireframeShader->bind();
	mWireframeShader->uploadAttrib("vin_flag", mWireframeFlags);

	updateVertexInfo();
	setShadeMode(getShadeMode());
}

void ViewerData::updateVertexInfo() {
	if (mMesh == nullptr) {
		return;
	}

	if (mMesh->isVertexPosDirty()) {
		const std::vector< Face* >& faces = mMesh->faces();
		int numFaces = faces.size();

		mMeshVertices = Eigen::Matrix3Xf::Zero(3, 3 * numFaces);
		mMeshPerFaceNormals = Eigen::Matrix3Xf::Zero(3, 3 * numFaces);// Update per-face normal
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			Face* f = faces[fidx];
			const Eigen::Vector3f& p0 = f->halfEdge()->start()->position();
			const Eigen::Vector3f& p1 = f->halfEdge()->end()->position();
			const Eigen::Vector3f& p2 = f->halfEdge()->next()->end()->position();

			mMeshVertices.col(fidx * 3 + 0) = p0;
			mMeshVertices.col(fidx * 3 + 1) = p1;
			mMeshVertices.col(fidx * 3 + 2) = p2;

			Eigen::Vector3f normal = ((p1 - p0).cross(p2 - p0)).normalized();
			mMeshPerFaceNormals.col(fidx * 3 + 0) = normal;
			mMeshPerFaceNormals.col(fidx * 3 + 1) = normal;
			mMeshPerFaceNormals.col(fidx * 3 + 2) = normal;

		}
		mMeshShader->bind();
		mMeshShader->uploadAttrib("vin_position", mMeshVertices);

		const std::vector< HEdge* >& hedges = mMesh->edges();
		int numHEdges = hedges.size();
		const std::vector< HEdge* > bhedges = mMesh->boundaryEdges();
		int numBHEdges = bhedges.size();

		mWireframeVertices = Eigen::Matrix3Xf::Zero(3, 2 * (numHEdges + numBHEdges));
		for (int eidx = 0; eidx < numHEdges; ++eidx) {
			HEdge* he = hedges[eidx];
			mWireframeVertices.col(eidx * 2 + 0) = he->start()->position();
			mWireframeVertices.col(eidx * 2 + 1) = he->end()->position();
		}
		for (int eidx = 0; eidx < numBHEdges; ++eidx) {
			HEdge* he = bhedges[eidx];
			mWireframeVertices.col(2 * numHEdges + eidx * 2 + 0) = he->start()->position();
			mWireframeVertices.col(2 * numHEdges + eidx * 2 + 1) = he->end()->position();
		}
		mWireframeShader->bind();
		mWireframeShader->uploadAttrib("vin_position", mWireframeVertices);

		mMesh->setVertexPosDirty(false);
	}

	if (mMesh->isVertexNormalDirty()) {
		const std::vector< Face* >& faces = mMesh->faces();
		int numFaces = faces.size();
		mMeshPerVertexNormals = Eigen::Matrix3Xf::Zero(3, 3 * numFaces);

		for (int fidx = 0; fidx < numFaces; ++fidx) {
			Face* f = faces[fidx];
			Vertex* v0 = f->halfEdge()->start();
			Vertex* v1 = f->halfEdge()->end();
			Vertex* v2 = f->halfEdge()->next()->end();

			mMeshPerVertexNormals.col(fidx * 3 + 0) = v0->normal();
			mMeshPerVertexNormals.col(fidx * 3 + 1) = v1->normal();
			mMeshPerVertexNormals.col(fidx * 3 + 2) = v2->normal();
		}
		mMesh->setVertexNormalDirty(false);
	}

	if (mMesh->isVertexColorDirty()) {
		const std::vector< Face* >& faces = mMesh->faces();
		int numFaces = faces.size();

		mMeshColors = Eigen::Matrix3Xf::Zero(3, 3 * numFaces);
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			Face* f = faces[fidx];
			Vertex* v0 = f->halfEdge()->start();
			Vertex* v1 = f->halfEdge()->end();
			Vertex* v2 = f->halfEdge()->next()->end();

			mMeshColors.col(fidx * 3 + 0) = v0->color();
			mMeshColors.col(fidx * 3 + 1) = v1->color();
			mMeshColors.col(fidx * 3 + 2) = v2->color();
		}

		mMeshShader->bind();
		mMeshShader->uploadAttrib("vin_color", mMeshColors);

		mMesh->setVertexColorDirty(false);
	}
}


// Internal global variables used for glfw event handling
static Viewer3D* _viewer3d;
static double _highdpi = 1;
static double _scroll_x = 0;
static double _scroll_y = 0;

static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier) {
	bool tw_used = _viewer3d->getScreen()->mouseButtonCallbackEvent(button, action, modifier);

	Viewer3D::MouseButton mb;
	if (button == GLFW_MOUSE_BUTTON_1)
		mb = Viewer3D::MouseButton::Left;
	else if (button == GLFW_MOUSE_BUTTON_2)
		mb = Viewer3D::MouseButton::Right;
	else //if (button == GLFW_MOUSE_BUTTON_3)
		mb = Viewer3D::MouseButton::Middle;

	if (action == GLFW_PRESS) {
		if (!tw_used) {
			_viewer3d->mouseDown(mb, modifier);
		}
	} else {
		// Always call mouse_up on up
		_viewer3d->mouseUp(mb, modifier);
	}
}

static void glfw_error_callback(int error, const char* description) {
	fputs(description, stderr);
}

static void glfw_char_mods_callback(GLFWwindow* window, unsigned int codepoint, int modifier) {
	if (!_viewer3d->getScreen()->charCallbackEvent(codepoint)) {
		_viewer3d->keyPressed(codepoint, modifier);
	}
}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (_viewer3d->getScreen()->keyCallbackEvent(key, scancode, action, modifier) == false) {
		if (action == GLFW_PRESS)
			_viewer3d->keyDown(key, modifier);
		else if (action == GLFW_RELEASE)
			_viewer3d->keyUp(key, modifier);
	}
}

static void glfw_window_size(GLFWwindow* window, int width, int height) {
	int w = width * _highdpi;
	int h = height * _highdpi;

	_viewer3d->postResize(w, h);
}

static void glfw_mouse_move(GLFWwindow* window, double x, double y) {
	if (_viewer3d->getScreen()->cursorPosCallbackEvent(x, y) == false) {
		_viewer3d->mouseMove(x * _highdpi, y * _highdpi);
	}
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y) {
	_scroll_x += x;
	_scroll_y += y;

	if (_viewer3d->getScreen()->scrollCallbackEvent(x, y) == false) {
		_viewer3d->mouseScroll(y);
	}
}

static void glfw_drop_callback(GLFWwindow* window, int count, const char** filenames) {
	_viewer3d->getScreen()->dropCallbackEvent(count, filenames);
}

static double get_seconds() {
	return std::chrono::duration< double >(std::chrono::system_clock::now().time_since_epoch()).count();
}

// -- Implementation of Viewer3D --
Viewer3D::Viewer3D() : mWindow(nullptr),
                       mNGui(nullptr),
                       mNScreen(nullptr),
                       mMesh(nullptr),
                       mViewerData(nullptr),
                       mDeformer(nullptr) {
	mWindowWidth = 1280;
	mWindowHeight = 800;
	mMouseCurrentX = 0;
	mMouseCurrentY = 0;

	mBgColor = Eigen::Vector4f(0.9f, 0.9f, 0.9f, 1.0f);
	clearMesh();
	mUseCotWeights = false;
	mSmoothScheme = SmoothScheme::Explicit;
	mHandleState = HandleState::Add;
	mSelectedHandle = 0;
	mDeformer = new Deformer();
}

Viewer3D::~Viewer3D() {
	if (mViewerData) {
		delete mViewerData;
	}
	mViewerData = nullptr;
	clearMesh();
	if (mDeformer) {
		delete mDeformer;
	}
	mDeformer = nullptr;
}

int Viewer3D::launch() {
	launchInit();
	launchRendering(true);
	launchShut();
	return EXIT_SUCCESS;
}

void Viewer3D::draw() {
	glClearColor(mBgColor[0], mBgColor[1], mBgColor[2], mBgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	mViewerData->draw();
	mNScreen->drawContents();
	mNScreen->drawWidgets();
}

bool Viewer3D::keyPressed(unsigned unicode_key, int modifier) {
	return false;
}

bool Viewer3D::keyDown(int key, int modifier) {
	return false;
}

bool Viewer3D::keyUp(int key, int modifier) {
	return false;
}

bool Viewer3D::mouseDown(MouseButton button, int modifier) {
	mMouseDownX = mMouseCurrentX;
	mMouseDownY = mMouseCurrentY;
	mMouseDownButton = button;
	if (mMouseDownButton == MouseButton::Right && mHandleState == HandleState::Move) {
		mSelectedHandle = selectHandle(mMouseDownX, mMouseDownY);
		std::cout << "Handle " << mSelectedHandle << " selected\n";
	}
	return true;
}

bool Viewer3D::mouseUp(MouseButton button, int modifier) {
	if (mMouseDownButton == MouseButton::Right) {
		if (mHandleState == HandleState::Add) {
			Eigen::Vector2f bboxMin(std::min(mMouseDownX, mMouseCurrentX),
			                        std::min(mMouseDownY, mMouseCurrentY));
			Eigen::Vector2f bboxMax(std::max(mMouseDownX, mMouseCurrentX),
			                        std::max(mMouseDownY, mMouseCurrentY));
			selectVertexByRect(bboxMin, bboxMax);
		} else if (mHandleState == HandleState::Move) {
			if (mMesh) {
				mMesh->computeVertexNormals();
				mMesh->setVertexPosDirty(true);
			}
		}
	}

	mMouseDownButton = MouseButton::None;
	return false;
}

bool Viewer3D::mouseMove(int mouse_x, int mouse_y) {
	int diffX = mMouseCurrentX - mouse_x;
	int diffY = mMouseCurrentY - mouse_y;

	int lastX = mMouseCurrentX;
	int lastY = mMouseCurrentY;

	mMouseCurrentX = mouse_x;
	mMouseCurrentY = mouse_y;

	if (mMouseDownButton == MouseButton::Left) {
		// Change viewpoints using the left mouse button
		if (mViewerData->getCamera()) {
			GLCamera* camera = mViewerData->getCamera();
			float orbitLen = (camera->getPosition() - mViewerData->getSceneCenter()).norm();
			camera->orbitRight(orbitLen, diffX / 4.0);
			camera->orbitDown(orbitLen, diffY / 4.0);
		}
	} else if (mMouseDownButton == MouseButton::Right) {
		if (diffX != 0 || diffY != 0) {
			moveHandle(lastX, lastY, mMouseCurrentX, mMouseCurrentY);
		}
	}
	return true;
}

bool Viewer3D::mouseScroll(float delta_y) {
	if (mViewerData->getCamera()) {
		mViewerData->getCamera()->moveForward(mViewerData->getSceneRadius() * delta_y / 10.0);
	}
	return true;
}

void Viewer3D::resize(int w, int h) {
	if (mWindow) {
		glfwSetWindowSize(mWindow, w / _highdpi, h / _highdpi);
	} else {
		postResize(w, h);
	}
}

void Viewer3D::postResize(int w, int h) {
	mWindowWidth = w;
	mWindowHeight = h;
	if (mViewerData) {
		mViewerData->setWindowSize(w, h);
	}
}

nanogui::Screen* Viewer3D::getScreen() {
	return mNScreen;
}

int Viewer3D::launchInit() {
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return EXIT_FAILURE;

	// Must disable the following lines in order to use opengl 1/2 on windows...
	glfwWindowHint(GLFW_SAMPLES, 8);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	mWindow = glfwCreateWindow(mWindowWidth, mWindowHeight, "Mesh Viewer", nullptr, nullptr);
	if (!mWindow) {
		glfwTerminate();
		return EXIT_FAILURE;
	}

	glfwMakeContextCurrent(mWindow);
	//#if defined(DEBUG) || defined(_DEBUG)
	int major, minor, rev;
	major = glfwGetWindowAttrib(mWindow, GLFW_CONTEXT_VERSION_MAJOR);
	minor = glfwGetWindowAttrib(mWindow, GLFW_CONTEXT_VERSION_MINOR);
	rev = glfwGetWindowAttrib(mWindow, GLFW_CONTEXT_REVISION);
	printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
	//printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
	//printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
	//#endif
	glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	mNScreen = new nanogui::Screen();
	mNScreen->initialize(mWindow, false);
	mNGui = new nanogui::FormHelper(mNScreen);

	_viewer3d = this;

	// Register callbacks
	glfwSetKeyCallback(mWindow, glfw_key_callback);
	glfwSetCursorPosCallback(mWindow, glfw_mouse_move);
	glfwSetWindowSizeCallback(mWindow, glfw_window_size);
	glfwSetMouseButtonCallback(mWindow, glfw_mouse_press);
	glfwSetScrollCallback(mWindow, glfw_mouse_scroll);
	glfwSetCharModsCallback(mWindow, glfw_char_mods_callback);
	glfwSetDropCallback(mWindow, glfw_drop_callback);

	// Handle retina displays (windows and mac)
	int width, height;
	glfwGetFramebufferSize(mWindow, &width, &height);
	int width_window, height_window;
	glfwGetWindowSize(mWindow, &width_window, &height_window);
	_highdpi = width / width_window;

	glfw_window_size(mWindow, width_window, height_window);

	// Init GUI
	mViewerData = new ViewerData();
	mViewerData->setWindowSize(mWindowWidth, mWindowHeight);

	mNGui->setFixedSize(Eigen::Vector2i(80, 20));
	mNGui->addWindow(Eigen::Vector2i(10, 10), "Control Panel");

	mNGui->addGroup("Mesh");
	mNGui->addButton("Load", [&]() { this->openDialogLoadMesh(); });

	mNGui->addGroup("Draw Options");
	mNGui->addVariable< ViewerData::ShadeMode >("Shading", [&](ViewerData::ShadeMode sm)
                                           {
	                                           mViewerData->setShadeMode(sm);
                                           }, [&]()
                                           {
	                                           return mViewerData->getShadeMode();
                                           })->setItems({"Flat", "Smooth", "None"});
	mNGui->addVariable< bool >("Wireframe", [&](bool checked)
                          {
	                          mViewerData->setUseWireframe(checked);
                          }, [&]()
                          {
	                          return mViewerData->useWireframe();
                          });
	mNGui->addVariable("Background", (nanogui::Color &)mBgColor);

	mNGui->addGroup("Smoothing");
	mNGui->addVariable< SmoothScheme >("Scheme", mSmoothScheme)
	     ->setItems({"Explicit", "Implicit"});
	mNGui->addVariable("Cot. Weights", mUseCotWeights);
	mNGui->addButton("Update", [&]() { smoothMesh(); });

	mNGui->addGroup("Deformation");

	mNGui->addVariable< HandleState >("Handle", mHandleState)
	     ->setItems({"Add", "Move"});
	mNGui->addButton("Build", [&]()
                {
	                buildDeformMat();
                });

	mNScreen->setVisible(true);
	mNScreen->performLayout();

	return EXIT_SUCCESS;
}

bool Viewer3D::launchRendering(bool loop) {
	// Rendering loop
	while (!glfwWindowShouldClose(mWindow)) {
		double tic = get_seconds();
		draw();

		glfwSwapBuffers(mWindow);
		glfwPollEvents();
		// In microseconds
		double duration = 1000000. * (get_seconds() - tic);
		const double min_duration = 1000000. / 30;
		if (duration < min_duration) {
			std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
		}

		if (!loop)
			return !glfwWindowShouldClose(mWindow);
	}
	return EXIT_SUCCESS;
}

void Viewer3D::launchShut() {
	if (mNGui) {
		delete mNGui;
	}
	mNScreen = nullptr;
	mNGui = nullptr;

	glfwDestroyWindow(mWindow);
	glfwTerminate();

	clearMesh();
}

void Viewer3D::openDialogLoadMesh() {
	std::string filename = igl::file_dialog_open();
	if (filename.length() == 0) {
		std::cout << __FUNCTION__ << ": invalid file name.\n";
		return;
	}

	clearMesh();

	mMesh = new Mesh();
	mMesh->loadMeshFile(filename);
	std::cout << "mesh " << filename << " loaded.\n";
	mMesh->computeVertexNormals();
	mViewerData->setMesh(mMesh, float(mWindowWidth) / float(mWindowHeight));

	std::vector< int > stats = mMesh->collectMeshStats();
	std::cout << "#vertices             = " << stats[0] << "\n";
	std::cout << "#half_edges           = " << stats[1] << "\n";
	std::cout << "#faces                = " << stats[2] << "\n";
	std::cout << "#boundary_loops       = " << stats[3] << "\n";
	std::cout << "#connected_components = " << stats[4] << "\n";
	std::cout << "#genus                = " << stats[5] << "\n\n";

}

void Viewer3D::clearMesh() {
	if (mMesh) {
		delete mMesh;
	}
	mMesh = nullptr;
}

void Viewer3D::smoothMesh() {
	if (mMesh == nullptr) {
		return;
	}

	switch (mSmoothScheme) {
	case SmoothScheme::Implicit:
		mMesh->implicitUmbrellaSmooth(mUseCotWeights);
		break;
	case SmoothScheme::Explicit:
	default:
		mMesh->umbrellaSmooth(mUseCotWeights);
		break;
	}
}

void Viewer3D::buildDeformMat() {
	if (mMesh == nullptr) {
		return;
	}
	mMesh->groupingVertexFlags();
	mDeformer->setMesh(mMesh);
}

void Viewer3D::clearHandles() {
	if (mMesh == nullptr) {
		return;
	}

	const std::vector< Vertex* >& vertices = mMesh->vertices();
	for (Vertex* vert : vertices) {
		vert->setFlag(0);
		vert->setColor(VCOLOR_BLUE);
	}
	mMesh->setVertexColorDirty(true);
}

void Viewer3D::selectVertexByRect(const Eigen::Vector2f& bboxMin,
                                  const Eigen::Vector2f& bboxMax) {
	// Select vertices using the right mouse button
	if (mMesh == nullptr || mViewerData->getCamera() == nullptr) {
		return;
	}

	const std::vector< Vertex* >& vertices = mMesh->vertices();
	Eigen::Matrix4f mvp = mViewerData->getCamera()->getTransformMat();
	Eigen::Vector4f viewport(0, 0, mWindowWidth, mWindowHeight);

	for (Vertex* vert : vertices) {
		Eigen::Vector3f projVert = gluProject(mvp, viewport, vert->position());
		if (projVert(0) >= 0 &&
		    projVert(0) < viewport(2) &&
		    projVert(1) >= 0 &&
		    projVert(1) < viewport(3) &&
		    projVert(0) > bboxMin(0) &&
		    projVert(0) < bboxMax(0) &&
		    projVert(1) > bboxMin(1) &&
		    projVert(1) < bboxMax(1)) {

			vert->setFlag(1);
			vert->setColor(VCOLOR_PURPLE);
		}
	}

	mMesh->setVertexColorDirty(true);
}

int Viewer3D::selectHandle(float mouseX, float mouseY) {
	if (mMesh == nullptr || mViewerData->getCamera() == nullptr) {
		return 0;
	}

	const std::vector< Vertex* >& vertices = mMesh->vertices();
	Eigen::Matrix4f mvp = mViewerData->getCamera()->getTransformMat();
	Eigen::Vector4f viewport(0, 0, mWindowWidth, mWindowHeight);

	float minDis = 0;
	int minIndex = -1;
	for (int vidx = 0; vidx < vertices.size(); ++vidx) {
		Vertex* vert = vertices[vidx];
		if (vert->flag() < 1) {
			continue;
		}

		Eigen::Vector3f v = gluProject(mvp, viewport, vert->position());
		float diffX = v(0) - mouseX;
		float diffY = v(1) - mouseY;
		float dis = diffX * diffX + diffY * diffY;
		if (dis < minDis || minIndex == -1) {
			minDis = dis;
			minIndex = vidx;
		}
	}
	return (minIndex != -1) ? vertices[minIndex]->flag() : 0;
}

void Viewer3D::moveHandle(float lastMouseX, float lastMouseY,
                          float currentMouseX, float currentMouseY) {
	if (mMesh == nullptr || mViewerData->getCamera() == nullptr || mSelectedHandle < 1) {
		return;
	}
	const std::vector< Vertex* >& vertices = mMesh->vertices();
	Eigen::Matrix4f mvp = mViewerData->getCamera()->getTransformMat();
	Eigen::Matrix4f mvpInv = mvp.inverse();
	Eigen::Vector4f viewport(0, 0, mWindowWidth, mWindowHeight);

	Eigen::Vector3f offset1 = gluUnproject(mvpInv, viewport,
	                                       Eigen::Vector3f(currentMouseX, currentMouseY, 0));
	Eigen::Vector3f offset2 = gluUnproject(mvpInv, viewport,
	                                       Eigen::Vector3f(lastMouseX, lastMouseY, 0));
	Eigen::Vector3f offset = (offset1 - offset2) * mViewerData->getSceneRadius() * 0.5;
	for (Vertex* vert : vertices) {
		if (vert->flag() == mSelectedHandle) {
			vert->setPosition(vert->position() + offset);
		}
	}

	if (mDeformer) {
		mDeformer->deform();
		mMesh->setVertexPosDirty(true);
	}
}
