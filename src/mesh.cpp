#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Sparse>
#include <queue>


HEdge::HEdge(bool b) {
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mFlag = false;
	mValid = true;
}

HEdge* HEdge::twin() const {
	return mTwin;
}

HEdge* HEdge::setTwin(HEdge* e) {
	mTwin = e;
	return mTwin;
}

HEdge* HEdge::prev() const {
	return mPrev;
}

HEdge* HEdge::setPrev(HEdge* e) {
	mPrev = e;
	return mPrev;
}

HEdge* HEdge::next() const {
	return mNext;
}

HEdge* HEdge::setNext(HEdge* e) {
	mNext = e;
	return mNext;
}

Vertex* HEdge::start() const {
	return mStart;
}

Vertex* HEdge::setStart(Vertex* v) {
	mStart = v;
	return mStart;
}

Vertex* HEdge::end() const {
	return mNext->start();
}

Face* HEdge::leftFace() const {
	return mFace;
}

Face* HEdge::setFace(Face* f) {
	mFace = f;
	return mFace;
}

bool HEdge::flag() const {
	return mFlag;
}

bool HEdge::setFlag(bool b) {
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const {
	return mBoundary;
}

bool HEdge::isValid() const {
	return mValid;
}

bool HEdge::setValid(bool b) {
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == nullptr) {
		mStart = nullptr;
		mNext = nullptr;
	} else {
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}

HEdge* OneRingHEdge::nextHEdge() {
	HEdge* ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart) {
		mNext = mNext->prev()->twin();
	} else {
		mNext = nullptr;
	}
	return ret;
}

OneRingVertex::OneRingVertex(const Vertex* v): ring(v) {
}

Vertex* OneRingVertex::nextVertex() {
	HEdge* he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f& v): mPosition(v), mHEdge(nullptr), mFlag(0) {
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z): mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}


const Eigen::Vector3f& Vertex::position() const {
	return mPosition;
}

const Eigen::Vector3f& Vertex::setPosition(const Eigen::Vector3f& p) {
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f& Vertex::normal() const {
	return mNormal;
}

const Eigen::Vector3f& Vertex::setNormal(const Eigen::Vector3f& n) {
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f& Vertex::color() const {
	return mColor;
}

const Eigen::Vector3f& Vertex::setColor(const Eigen::Vector3f& c) {
	mColor = c;
	return mColor;
}

HEdge* Vertex::halfEdge() const {
	return mHEdge;
}

HEdge* Vertex::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

int Vertex::index() const {
	return mIndex;
}

int Vertex::setIndex(int i) {
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const {
	return mFlag;
}

int Vertex::setFlag(int f) {
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const {
	return mValid;
}

bool Vertex::setValid(bool b) {
	mValid = b;
	return mValid;
}

bool Vertex::isBoundary() const {
	OneRingHEdge ring(this);
	HEdge* curr = nullptr;
	while (curr = ring.nextHEdge()) {
		if (curr->isBoundary()) {
			return true;
		}
	}
	return false;
}

int Vertex::valence() const {
	int count = 0;
	OneRingVertex ring(this);
	Vertex* curr = nullptr;
	while (curr = ring.nextVertex()) {
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mValid(true) {
}

HEdge* Face::halfEdge() const {
	return mHEdge;
}

HEdge* Face::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

bool Face::isBoundary() const {
	HEdge* curr = mHEdge;
	do {
		if (curr->twin()->isBoundary()) {
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const {
	return mValid;
}

bool Face::setValid(bool b) {
	mValid = b;
	return mValid;
}

Mesh::Mesh() {
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
}

Mesh::~Mesh() {
	clear();
}

const std::vector< HEdge* >& Mesh::edges() const {
	return mHEdgeList;
}

const std::vector< HEdge* >& Mesh::boundaryEdges() const {
	return mBHEdgeList;
}

const std::vector< Vertex* >& Mesh::vertices() const {
	return mVertexList;
}

const std::vector< Face* >& Mesh::faces() const {
	return mFaceList;
}


bool Mesh::isVertexPosDirty() const {
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b) {
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const {
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b) {
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const {
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b) {
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename) {
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	if (iglFlag) {
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list
		for (int vidx = 0; vidx < numVertices; ++vidx) {
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
			                                 mVertexMat(vidx, 1),
			                                 mVertexMat(vidx, 2)));
		}
		// Fill in the face list
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
		}

		std::vector< HEdge* > hedgeList;
		for (int i = 0; i < mBHEdgeList.size(); ++i) {
			if (mBHEdgeList[i]->start()) {
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i) {
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	} else {
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge* e1, HEdge* e2) {
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge* e1, HEdge* e2) {
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face* f, HEdge* e) {
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3) {
	Face* face = new Face();

	HEdge* hedge[3];
	HEdge* bhedge[3]; // Boundary half-edges
	Vertex* vert[3];

	for (int i = 0; i < 3; ++i) {
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i) {
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i) {
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i) {
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) {
		Vertex* start = bhedge[i]->start();
		Vertex* end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start) {
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr); // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i) {
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

Eigen::Vector3f Mesh::initBboxMin() const {
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const {
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags() {
	// Init to 255
	for (Vertex* vert : mVertexList) {
		if (vert->flag() != 0) {
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector< Vertex* > tmpList;
	for (Vertex* vert : mVertexList) {
		if (vert->flag() == 255) {
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty()) {
				Vertex* v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex* v2 = orv.nextVertex()) {
					if (v2->flag() == 255) {
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

void Mesh::clear() {
	for (int i = 0; i < mHEdgeList.size(); ++i) {
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i) {
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i) {
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector< int > Mesh::collectMeshStats() {
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Collect mesh information as listed above.
	/**********************************************/

	V = mVertexList.size();
	E = mHEdgeList.size() + mBHEdgeList.size();
	F = mFaceList.size();
	B = this->countBoundaryLoops();
	C = this->countConnectedComponents();
	G = C - (V - E/2 + F + B) / 2;

	/*====== Programming Assignment 0 ======*/

	std::vector< int > stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

int Mesh::countBoundaryLoops() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/

	for (int i = 0; i < mBHEdgeList.size(); ++i)
		mBHEdgeList[i]->setFlag(false);

	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		HEdge* it = mBHEdgeList[i];
		if (it->flag())
			continue;

		it->setFlag(true);
		HEdge* startHEdge = it;
		HEdge* e = it;
		while (e->next() != startHEdge) {
			e = e->next();
			e->setFlag(true);
		}
		++count;
	}

	for (int i = 0; i < mBHEdgeList.size(); ++i)
		mBHEdgeList[i]->setFlag(false);

	/*====== Programming Assignment 0 ======*/

	return count;
}

int Mesh::countConnectedComponents() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/

	for (HEdge* it : mHEdgeList)
		it->setFlag(false);

	for (HEdge* it : mHEdgeList) {
		if (it->flag())
			continue;

		std::queue< HEdge* > q;
		it->setFlag(true);
		q.push(it);
		while(!q.empty()) {
			HEdge* front = q.front();
			q.pop();

			if (front->next() && !front->next()->flag()) {
				q.push(front->next());
				front->next()->setFlag(true);
			}
			if (front->twin() && !front->twin()->flag()) {
				q.push(front->twin());
				front->twin()->setFlag(true);
			}
		}
		++count;
	}

	for (HEdge* it : mHEdgeList)
		it->setFlag(false);

	/*====== Programming Assignment 0 ======*/

	return count;
}

void Mesh::computeVertexNormals() {
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/

	for (Vertex* it : mVertexList) {
		OneRingVertex r = OneRingVertex(it);
		std::vector< Vertex* > vertices;
		Vertex* v;
		while (v = r.nextVertex())
			vertices.push_back(v);

		std::vector< Eigen::Vector3f > normals;
		std::vector< float > areas;
		for (int i = 0; i < vertices.size(); ++i) {
			Eigen::Vector3f v1 = vertices[i]->position() - it->position();
			Eigen::Vector3f v2 = vertices[(i+1) % vertices.size()]->position() - it->position();

			Eigen::Vector3f normal = v1.cross(v2);
			float area = normal.norm() / 2.0;

			normals.push_back(normal);
			areas.push_back(area);
		}

		Eigen::Vector3f n(0, 0, 0);
		float d = 0;
		for (int i = 0; i < areas.size(); ++i) {
			n += normals[i] * areas[i];
			d += areas[i];
		}
		Eigen::Vector3f vertexNormal(n / d);
		it->setNormal(vertexNormal);
	}

	/*====== Programming Assignment 0 ======*/

	// Notify mesh shaders
	setVertexNormalDirty(true);
}


void Mesh::umbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 1: Implement the cotangent weighting
		/* scheme for explicit mesh smoothing.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/

		double lamda = 1;
		int numVertex = mVertexList.size();

		//typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
		//typedef Eigen::Triplet<double> T;
		std::vector<Eigen::Triplet<double>> tripletList;
		tripletList.reserve(7 * numVertex);

		for (int p = 0; p < numVertex; ++p) {
			Vertex* it = mVertexList[p];
			//std::cout << it->position() << std::endl;

		  OneRingVertex r = OneRingVertex(it);
			std::vector< Vertex* > vertices;
			Vertex* v;
			while (v = r.nextVertex())
				vertices.push_back(v);

			std::vector< double > weights;
			double weightSum = 0;

			for (int i = 0; i < vertices.size(); ++i) {
				Vertex* curr = vertices[i];
				Vertex* prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
				Vertex* next = vertices[(i + 1) % vertices.size()];

				Eigen::Vector3f prev1 = it->position() - prev->position();
				Eigen::Vector3f prev2 = curr->position() - prev->position();
				double prevDot = prev1.dot(prev2);
				double prevCross = (prev1.cross(prev2)).norm();
				double prevCot = prevDot / prevCross;

				Eigen::Vector3f next1 = it->position() - next->position();
				Eigen::Vector3f next2 = curr->position() - next->position();
				double nextDot = next1.dot(next2);
				double nextCross = (next1.cross(next2)).norm();
				double nextCot = nextDot / nextCross;

				double weight = prevCot + nextCot;
				weights.push_back(weight);
				weightSum += weight;
				//std::cout << weight << std::endl;
			}

			for (int t = 0; t < weights.size(); ++t) {
				//std::cout << "w0:" << weights[t];
				weights[t] /= weightSum;
				//std::cout << "\nw1:" << weights[t];
			}
			//for (double w : weights) std::cout << "\nupdatedW:" << w;
			//std::cout << "weightSum:" << weightSum << std::endl;
			for (int k = 0; k < vertices.size(); ++k) {
				int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
		    if (index == numVertex)
		    	exit(-1);

			  tripletList.push_back(Eigen::Triplet<double>(p, index, weights[k]));
				//std::cout << "\nappendW:" << weights[k];
			}

			int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
			if (index == numVertex)
				exit(-1);

			tripletList.push_back(Eigen::Triplet<double>(p, index, -1));
		}

		Eigen::SparseMatrix< double > matL(numVertex, numVertex);
		matL.setFromTriplets(tripletList.begin(), tripletList.end());
		//std::cout << "matL\n" << matL << '\n';
		Eigen::MatrixXd matP(1, 3);
		matP.resize(numVertex, 3);
		for (int i = 0; i < numVertex; ++i)
			matP.row(i) = mVertexList[i]->position().transpose().cast< double >();
		//std::cout << "matP\n" << matP << '\n';
		Eigen::MatrixXd matX(1, 3);
		matX.resize(numVertex, 3);
		matX = lamda * matL * matP;

		for (int i = 0; i < numVertex; ++i) {
			//std::cout << mVertexList[i]->position() << '	';
			mVertexList[i]->setPosition(mVertexList[i]->position() + matX.row(i).transpose().cast< float >());
			//std::cout << "matXrow: " << matX.row(i) << std::endl;
			//std::cout << mVertexList[i]->position() << '	';
		}
		//std::cout << '/n/n';
		//for (int i = 0; i < mVertexList.size(); ++i) std::cout << mVertexList[i]->position() << '	';
		//std::cout << '/n/n';
		//std::cout << "matL\n" << matL << '\n';
		//std::cout << "matX\n" << matX << '\n';
	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the uniform weighting
		/* scheme for explicit mesh smoothing.
		/**********************************************/

		double lamda = 1;
		int numVertex = mVertexList.size();

		std::vector<Eigen::Triplet<double>> tripletList;
		tripletList.reserve(7 * numVertex);

		for (int p = 0; p < numVertex; ++p) {
			Vertex* it = mVertexList[p];

		  OneRingVertex r = OneRingVertex(it);
			std::vector< Vertex* > vertices;
			Vertex* v;
			while (v = r.nextVertex())
				vertices.push_back(v);

			for (int k = 0; k < vertices.size(); ++k) {
				int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
		    if (index == numVertex)
		    	exit(-1);

			  tripletList.push_back(Eigen::Triplet<double>(p, index, (double)(1.0 / vertices.size()))); // only change
			}

			int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
			if (index == numVertex)
				exit(-1);

			tripletList.push_back(Eigen::Triplet<double>(p, index, -1));
		}

		Eigen::SparseMatrix< double > matL(numVertex, numVertex);
		matL.setFromTriplets(tripletList.begin(), tripletList.end());

		Eigen::MatrixXd matP(1, 3);
		matP.resize(numVertex, 3);
		for (int i = 0; i < numVertex; ++i)
			matP.row(i) = mVertexList[i]->position().transpose().cast< double >();

		Eigen::MatrixXd matX(1, 3);
		matX.resize(numVertex, 3);
		matX = lamda * matL * matP;

		for (int i = 0; i < numVertex; ++i)
			mVertexList[i]->setPosition(mVertexList[i]->position() + matX.row(i).transpose().cast< float >());
	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}

void Mesh::implicitUmbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/

	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	auto fnConjugateGradient = [](const Eigen::SparseMatrix< float >& A,
	                              const Eigen::VectorXf& b,
	                              int maxIterations,
	                              float errorTolerance,
	                              Eigen::VectorXf& x)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Params:
		/*  A:
		/*  b:
		/*  maxIterations:	Max number of iterations
		/*  errorTolerance: Error tolerance for the early stopping condition
		/*  x:				Stores the final solution, but should be initialized.
		/**********************************************/
		/*
		/* Step 1: Implement the conjugate gradient
		/* method.
		/* Hint: https://en.wikipedia.org/wiki/Conjugate_gradient_method
		/**********************************************/

		Eigen::BiCGSTAB< Eigen::SparseMatrix<float> > solver;
		solver.setMaxIterations(maxIterations);
		solver.setTolerance(errorTolerance);
		solver.compute(A);
		x = solver.solve(b);
		//std::cout << "#iterations:     " << solver.iterations() << std::endl;
		//std::cout << "estimated error: " << solver.error()      << std::endl;
	};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the cotangent weighting
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/

		float lamda = 1;
		int numVertex = mVertexList.size();

		std::vector<Eigen::Triplet<float>> tripletList;
		tripletList.reserve(7 * numVertex);

		for (int p = 0; p < numVertex; ++p) {
			Vertex* it = mVertexList[p];

			OneRingVertex r = OneRingVertex(it);
			std::vector< Vertex* > vertices;
			Vertex* v;
			while (v = r.nextVertex())
				vertices.push_back(v);

			std::vector< float > weights;
			float weightSum = 0;

			for (int i = 0; i < vertices.size(); ++i) {
				Vertex* curr = vertices[i];
				Vertex* prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
				Vertex* next = vertices[(i + 1) % vertices.size()];

				Eigen::Vector3f prev1 = it->position() - prev->position();
				Eigen::Vector3f prev2 = curr->position() - prev->position();
				float prevDot = prev1.dot(prev2);
				float prevCross = (prev1.cross(prev2)).norm();
				float prevCot = prevDot / prevCross;

				Eigen::Vector3f next1 = it->position() - next->position();
				Eigen::Vector3f next2 = curr->position() - next->position();
				float nextDot = next1.dot(next2);
				float nextCross = (next1.cross(next2)).norm();
				float nextCot = nextDot / nextCross;

				float weight = prevCot + nextCot;
				weights.push_back(weight);
				weightSum += weight;
			}

			for (int t = 0; t < weights.size(); ++t)
				weights[t] /= weightSum;

			for (int k = 0; k < vertices.size(); ++k) {
				int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
				if (index == numVertex)
					exit(-1);

				tripletList.push_back(Eigen::Triplet<float>(p, index, weights[k]));
			}

			int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
			if (index == numVertex)
				exit(-1);

			tripletList.push_back(Eigen::Triplet<float>(p, index, -1));
		}

		Eigen::SparseMatrix< float > matL(numVertex, numVertex);
		matL.setFromTriplets(tripletList.begin(), tripletList.end());

		Eigen::MatrixXf matP;
		matP.resize(numVertex, 3);
		for (int i = 0; i < numVertex; ++i)
			matP.row(i) = mVertexList[i]->position().transpose();

		Eigen::MatrixXf matX;
		matX.resize(numVertex, 3);

		Eigen::SparseMatrix< float > matA(numVertex, numVertex);
		Eigen::SparseMatrix< float > matI(numVertex, numVertex);
		matI.setIdentity();
		matA = matI - lamda * matL;
		// Eigen::MatrixXf::Identity(numVertex, numVertex)

		Eigen::VectorXf vp(Eigen::Map< Eigen::VectorXf >(matP.col(0).data(), numVertex));
		Eigen::VectorXf vx(numVertex);
		for (int r = 0; r < 3; ++r) {
			fnConjugateGradient(matA, vp, 1000, 1e-10, vx);
			matX.col(r) = vx;
			if (r != 2)
				vp = matP.col(r+1);
		}

		for (int i = 0; i < numVertex; ++i)
			mVertexList[i]->setPosition(matX.row(i).transpose());

	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 3: Implement the uniform weighting
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/

		float lamda = 1;
		int numVertex = mVertexList.size();

		std::vector<Eigen::Triplet<float>> tripletList;
		tripletList.reserve(7 * numVertex);

		for (int p = 0; p < numVertex; ++p) {
			Vertex* it = mVertexList[p];

		  OneRingVertex r = OneRingVertex(it);
			std::vector< Vertex* > vertices;
			Vertex* v;
			while (v = r.nextVertex())
				vertices.push_back(v);

			for (int k = 0; k < vertices.size(); ++k) {
				int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
		    if (index == numVertex)
		    	exit(-1);

			  tripletList.push_back(Eigen::Triplet<float>(p, index, (float)(1.0 / vertices.size())));
			}

			int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
			if (index == numVertex)
				exit(-1);

			tripletList.push_back(Eigen::Triplet<float>(p, index, -1));
		}

		Eigen::SparseMatrix< float > matL(numVertex, numVertex);
		matL.setFromTriplets(tripletList.begin(), tripletList.end());

		Eigen::MatrixXf matP;
		matP.resize(numVertex, 3);
		for (int i = 0; i < numVertex; ++i)
			matP.row(i) = mVertexList[i]->position().transpose();

		Eigen::MatrixXf matX;
		matX.resize(numVertex, 3);

		Eigen::SparseMatrix< float > matA(numVertex, numVertex);
		Eigen::SparseMatrix< float > matI(numVertex, numVertex);
		matI.setIdentity();
		matA = matI - lamda * matL;
		// Eigen::MatrixXf::Identity(numVertex, numVertex)

		Eigen::VectorXf vp(Eigen::Map< Eigen::VectorXf >(matP.col(0).data(), numVertex));
		Eigen::VectorXf vx(numVertex);
		for (int r = 0; r < 3; ++r) {
			fnConjugateGradient(matA, vp, 10000, 0.00000000001, vx);
			matX.col(r) = vx;
			if (r != 2)
				vp = matP.col(r+1);
		}

		for (int i = 0; i < numVertex; ++i)
			mVertexList[i]->setPosition(matX.row(i).transpose());
	}
	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}
