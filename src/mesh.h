#ifndef MESH_H
#define MESH_H
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "util.h"

// Classes
class HEdge;
class Vertex;
class Face;
class Mesh;

// Class Half Edge
class HEdge {
public:
	HEdge(bool b = false);

	HEdge* twin() const;
	HEdge* setTwin(HEdge* e);

	HEdge* prev() const;
	HEdge* setPrev(HEdge* e);

	HEdge* next() const;
	HEdge* setNext(HEdge* e);

	Vertex* start() const;
	Vertex* setStart(Vertex* v);

	Vertex* end() const;

	Face* leftFace() const;
	Face* setFace(Face* f);

	bool flag() const;
	bool setFlag(bool b);

	bool isBoundary() const;

	bool isValid() const;
	bool setValid(bool b);

private:
	// Twin/previous/next half edges
	HEdge* mTwin;
	HEdge* mPrev;
	HEdge* mNext;

	// Start vertex
	Vertex* mStart;
	// Left face
	Face* mFace;

	// Flag for boundary edge
	bool mBoundary;

	/*
	 * It's for counting boundary loops.
	 * You can use it freely for marking in boundary loop counting
	 * and connected component counting
	 */
	bool mFlag;

	bool mValid;
};

// Class OneRingHEdge
// This class is used for accessing the neighboring HALF EDGES
// of a given vertex, please see Vertex::isBoundary() for its usage
class OneRingHEdge {
public:
	OneRingHEdge(const Vertex* v);

	// Iterator
	HEdge* nextHEdge();

private:
	HEdge* mStart;
	HEdge* mNext;
};

// Class OneRingVertex
// This class is used for accessing the neighboring VERTICES
// of a given vertex, please see Vertex::valence() for its usage
class OneRingVertex {
public:
	OneRingVertex(const Vertex* v);

	// Iterator
	Vertex* nextVertex();

private:
	OneRingHEdge ring;
};

// Class Vertex
class Vertex {
public:
	Vertex();
	Vertex(const Eigen::Vector3f& v);
	Vertex(float x, float y, float z);

	const Eigen::Vector3f& position() const;
	const Eigen::Vector3f& setPosition(const Eigen::Vector3f& p);

	const Eigen::Vector3f& normal() const;
	const Eigen::Vector3f& setNormal(const Eigen::Vector3f& n);

	const Eigen::Vector3f& color() const;
	const Eigen::Vector3f& setColor(const Eigen::Vector3f& c = VCOLOR_BLUE);

	HEdge* halfEdge() const;
	HEdge* setHalfEdge(HEdge* he);

	int index() const;
	int setIndex(int i);

	int flag() const;
	int setFlag(int f);

	bool isValid() const;
	bool setValid(bool b);

	bool isBoundary() const;

	int valence() const;

public:
	// For reading object only, DO NOT use it in other places
	std::vector< HEdge* > adjHEdges;

private:
	// Position (x,y,z) in space
	Eigen::Vector3f mPosition;
	// Normal vector for smooth shading rendering
	Eigen::Vector3f mNormal;
	// Color value
	Eigen::Vector3f mColor;

	// One of half edge starts with this vertex
	HEdge* mHEdge;
	// Original vertex index, DO NOT UPDATE IT!
	int mIndex;
	// 0 for moved, 1 for constrained
	int mFlag;
	bool mValid;
};

// Class Face
class Face {
public:
	Face();

	HEdge* halfEdge() const;
	HEdge* setHalfEdge(HEdge* he);

	bool isBoundary() const;

	bool isValid() const;
	bool setValid(bool b);

private:
	// Adjacent half-edge
	HEdge* mHEdge;
	bool mValid;
};

class Mesh {

public:
	Mesh();
	~Mesh();

	// Access functions
	const std::vector< HEdge* >& edges() const;
	const std::vector< HEdge* >& boundaryEdges() const;
	const std::vector< Vertex* >& vertices() const;
	const std::vector< Face* >& faces() const;

	// Flags
	bool isVertexPosDirty() const;
	void setVertexPosDirty(bool b);
	bool isVertexNormalDirty() const;
	void setVertexNormalDirty(bool b);
	bool isVertexColorDirty() const;
	void setVertexColorDirty(bool b);

	// Functions for loading obj files,
	// You DO NOT need to understand or use them.
	bool loadMeshFile(const std::string filename);
	void addFace(int v1, int v2, int v3);
	Eigen::Vector3f initBboxMin() const;
	Eigen::Vector3f initBboxMax() const;

	void groupingVertexFlags();
	void clear();

	/************************************************************************/

	/*====== Programming Assignment 0 ======*/
	std::vector< int > collectMeshStats();
	// Additional helper functions
	int countBoundaryLoops();
	int countConnectedComponents();

	void computeVertexNormals();
	/*====== Programming Assignment 0 ======*/

	/*====== Programming Assignment 1 ======*/
	void umbrellaSmooth(bool cotangentWeights = true);
	void implicitUmbrellaSmooth(bool cotangentWeights = true);
	/*====== Programming Assignment 1 ======*/

	/************************************************************************/

private:
	// List of NON-boundary half edges
	std::vector< HEdge* > mHEdgeList;
	// List of boundary half egdes
	std::vector< HEdge* > mBHEdgeList;
	// List of vertices
	std::vector< Vertex* > mVertexList;
	// List of faces
	std::vector< Face* > mFaceList;

	// Some flags you DO NOT need to care about
	bool mVertexPosFlag;
	bool mVertexNormalFlag;
	bool mVertexColorFlag;

	/* DO NOT use the following variables in YOUR CODE */
	// Original mesh information
	Eigen::MatrixX3f mVertexMat;
	Eigen::MatrixX3i mFaceMat;
};

#endif // MESH_H
