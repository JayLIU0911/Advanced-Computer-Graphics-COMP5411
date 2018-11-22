#include "deformer.h"
#include <iostream>

Deformer::Deformer() : mMesh(nullptr),
                       mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}


void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/

  // const std::vector< Vertex* > mVertexList = mMesh->vertices();
  // int numVertex = mVertexList.size();
  // int numRoi = mRoiList.size();
  // std::vector<Eigen::Triplet<float>> tripletList;
  // tripletList.reserve(7 * numVertex + numRoi);
  //
  // for (int p = 0; p < numVertex; ++p) {
  //   Vertex* it = mVertexList[p];
  //
  //   OneRingVertex r = OneRingVertex(it);
  //   std::vector< Vertex* > vertices;
  //   Vertex* v;
  //   while (v = r.nextVertex())
  //     vertices.push_back(v);
  //
  //   std::vector< float > weights;
  //   float weightSum = 0;
  //
  //   for (int i = 0; i < vertices.size(); ++i) {
  //     Vertex* curr = vertices[i];
  //     Vertex* prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
  //     Vertex* next = vertices[(i + 1) % vertices.size()];
  //
  //     Eigen::Vector3f prev1 = it->position() - prev->position();
  //     Eigen::Vector3f prev2 = curr->position() - prev->position();
  //     float prevDot = prev1.dot(prev2);
  //     float prevCross = (prev1.cross(prev2)).norm();
  //     float prevCot = prevDot / prevCross;
  //
  //     Eigen::Vector3f next1 = it->position() - next->position();
  //     Eigen::Vector3f next2 = curr->position() - next->position();
  //     float nextDot = next1.dot(next2);
  //     float nextCross = (next1.cross(next2)).norm();
  //     float nextCot = nextDot / nextCross;
  //
  //     float weight = prevCot + nextCot;
  //     weights.push_back(weight);
  //     weightSum += weight;
  //   }
  //
  //   for (int t = 0; t < weights.size(); ++t)
  //     weights[t] /= weightSum;
  //
  //   for (int k = 0; k < vertices.size(); ++k) {
  //     int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
  //     if (index == numVertex)
  //       exit(-1);
  //
  //     tripletList.push_back(Eigen::Triplet<float>(p, index, weights[k]));
  //   }
  //
  //   int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
  //   if (index == numVertex)
  //     exit(-1);
  //
  //   tripletList.push_back(Eigen::Triplet<float>(p, index, -1));
  // }
  //
  // Eigen::SparseMatrix< double > matA(numVertex + numRoi, numVertex);
  //
  // for (int p = 0; p < numRoi; ++p) {
  //   int index = std::find(mVertexList.begin(), mVertexList.end(), mRoiList[p]) - mVertexList.begin();
  //   tripletList.push_back(Eigen::Triplet<float>(numVertex + p, index, -1));
  // }
  //
  // matA.setFromTriplets(tripletList.begin(), tripletList.end());
  // matA = (-1) * matA;
  //
  // // std::cout << "triplet set\n";
  //
  // Eigen::MatrixXd matP(1, 3);
  // matP.resize(numVertex, 3);
  // for (int i = 0; i < numVertex; ++i)
  //   matP.row(i) = mVertexList[i]->position().transpose().cast< double >();
  //
  // Eigen::MatrixXd matSigma(1, 3);
  // matSigma.resize(numVertex + numRoi, 3);
  // matSigma = matA * matP;
  //
  // sigma.resize(numVertex + numRoi, 3);
  // sigma = matSigma;
  //
  // AT.resize(numVertex, numVertex + numRoi);
  // AT = matA.transpose();
  //
  // Eigen::SparseMatrix< double > systemMat;
  // systemMat.resize(numVertex, numVertex);
  // systemMat = AT * matA;


  const std::vector< Vertex* > mVertexList = mMesh->vertices();
  int numVertex = mVertexList.size();
  int numRoi = mRoiList.size();
  std::vector<Eigen::Triplet<float>> tripletList;
  tripletList.reserve((7 * numVertex + numRoi) * 3); //

  std::vector<Eigen::Triplet<float>> tripletT;
  tripletT.reserve(3 * 21 * numVertex);

  for (int p = 0; p < numVertex; ++p) {
    Vertex* it = mVertexList[p];

    OneRingVertex r = OneRingVertex(it);
    std::vector< Vertex* > vertices;
    Vertex* v;
    while (v = r.nextVertex())
      vertices.push_back(v);

    std::vector< float > weights;
    float weightSum = 0;

    int valence = vertices.size();

    for (int i = 0; i < valence; ++i) {
      Vertex* curr = vertices[i];
      Vertex* prev = vertices[(i - 1 + valence) % valence];
      Vertex* next = vertices[(i + 1) % valence];

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

    for (int k = 0; k < valence; ++k) {
      int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[k]) - mVertexList.begin();
      if (index == numVertex)
        exit(-1);

      tripletList.push_back(Eigen::Triplet<float>(p, index, weights[k]));
      tripletList.push_back(Eigen::Triplet<float>(p + numVertex, index + numVertex, weights[k])); //
      tripletList.push_back(Eigen::Triplet<float>(p + numVertex * 2, index + numVertex * 2, weights[k])); //
    }

    int index = std::find(mVertexList.begin(), mVertexList.end(), it) - mVertexList.begin();
    if (index == numVertex)
      exit(-1);

    tripletList.push_back(Eigen::Triplet<float>(p, index, -1));
    tripletList.push_back(Eigen::Triplet<float>(p + numVertex, index + numVertex, -1)); //
    tripletList.push_back(Eigen::Triplet<float>(p + numVertex * 2, index + numVertex * 2, -1)); //
  }

  Eigen::SparseMatrix< float > matA(3 * (numVertex + numRoi), 3 * numVertex); //
	Eigen::SparseMatrix< float > matT(3 * (numVertex + numRoi), 3 * numVertex); //

  sigma.resize((numVertex + numRoi) * 3);
  sigma.setZero();

  for (int p = 0; p < numRoi; ++p) {
    int index = std::find(mVertexList.begin(), mVertexList.end(), mRoiList[p]) - mVertexList.begin();
    tripletList.push_back(Eigen::Triplet<float>(numVertex * 3 + p, index, -1));
    tripletList.push_back(Eigen::Triplet<float>(p + numVertex * 3 + numRoi, index + numVertex, -1)); //
    tripletList.push_back(Eigen::Triplet<float>(p + numVertex * 3 + numRoi * 2, index + numVertex * 2, -1)); //

    //
    sigma[p + numVertex * 3] = mRoiList[p]->position()[0];
    sigma[p + numVertex * 3 + numRoi] = mRoiList[p]->position()[1];
    sigma[p + numVertex * 3 + numRoi * 2] = mRoiList[p]->position()[2];
  }

  matA.setFromTriplets(tripletList.begin(), tripletList.end());
  matA = (-1) * matA;

  // std::cout << "triplet set\n";
//
  Eigen::VectorXf vecP;
  vecP.resize(numVertex * 3);
  for (int i = 0; i < numVertex; ++i) {
    vecP[i] = mVertexList[i]->position()[0];
    vecP[i + numVertex] = mVertexList[i]->position()[1];
    vecP[i + numVertex * 2] = mVertexList[i]->position()[2];
  }

  Eigen::VectorXf vecSigma;
  vecSigma.resize((numVertex + numRoi) * 3);
  vecSigma = matA * vecP;

  // sigma.resize((numVertex + numRoi) * 3);
  // sigma = vecSigma;

  for (int d = 0; d < numVertex; ++d) {
    Eigen::MatrixXf D;
    D.resize(3, 7);
    D.setZero();

    float deltaX = vecSigma[d];
    float deltaY = vecSigma[d + numVertex];
    float deltaZ = vecSigma[d + numVertex * 2];

    D(0, 0) = D(1, 3) = deltaX;
    D(1, 0) = D(2, 1) = deltaY;
    D(0, 2) = D(2, 0) = deltaZ;

    D(2, 2) = (-1) * deltaX;
    D(0, 3) = (-1) * deltaY;
    D(1, 1) = (-1) * deltaZ;

    D(0, 4) = D(1, 5) = D(2, 6) = 1;

// C
    Vertex* it = mVertexList[d];

    OneRingVertex r = OneRingVertex(it);
    std::vector< Vertex* > vertices;
    Vertex* v;
    while (v = r.nextVertex())
      vertices.push_back(v);

    int valence = vertices.size();

    Eigen::MatrixXf C;
    C.resize(valence * 3 + 3, 7);
    C.setZero();

    float x = it->position()[0];
    float y = it->position()[1];
    float z = it->position()[2];

    C(0, 0) = x;
    C(0, 2) = z;
    C(0, 3) = -y;
    C(0, 4) = 1;

    C(1, 0) = y;
    C(1, 1) = -z;
    C(1, 3) = x;
    C(1, 5) = 1;

    C(2, 0) = z;
    C(2, 1) = y;
    C(2, 2) = -x;
    C(2, 6) = 1;

    for (int i = 0; i < valence; ++i) {
      Vertex* curr = vertices[i];

      x = curr->position()(0);
      y = curr->position()(1);
      z = curr->position()(2);

      C((i + 1) * 3, 0) = x;
      C((i + 1) * 3, 2) = z;
      C((i + 1) * 3, 3) = -y;
      C((i + 1) * 3, 4) = 1;

      C((i + 1) * 3 + 1, 3) = x;
      C((i + 1) * 3 + 1, 0) = y;
      C((i + 1) * 3 + 1, 1) = -z;
      C((i + 1) * 3 + 1, 5) = 1;

      C((i + 1) * 3 + 2, 0) = z;
      C((i + 1) * 3 + 2, 1) = y;
      C((i + 1) * 3 + 2, 2) = -x;
      C((i + 1) * 3 + 2, 6) = 1;
    }

    Eigen::MatrixXf T;
    T.resize(3, valence * 3 + 3);
    T = D * (C.transpose() * C).inverse() * C.transpose();

    for (int z = 0; z < 3; ++z) {
      tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, d, T(z, 0)));
      tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, d + numVertex, T(z, 1)));
      tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, d + numVertex * 2, T(z, 2)));

      for (int i = 0; i < valence; ++i) {
        int index = std::find(mVertexList.begin(), mVertexList.end(), vertices[i]) - mVertexList.begin();
        tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, index, T(z, (i + 1) * 3)));
        tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, index + numVertex, T(z, (i + 1) * 3 + 1)));
        tripletT.push_back(Eigen::Triplet<float>(d + numVertex * z, index + numVertex * 2, T(z, (i + 1) * 3 + 2)));
      }
    }
  }

  matT.setFromTriplets(tripletT.begin(), tripletT.end());

  matA = matA - matT;

  // std::cout << matA << "\n\n";
  // std::cout << matT << "\n\n";

  AT.resize(3 * numVertex, 3 * (numVertex + numRoi));
  AT = matA.transpose();

  Eigen::SparseMatrix< double > systemMat;
  systemMat.resize(numVertex, numVertex);
  systemMat = (AT * matA).cast< double >();

  // std::cout << "systemMat set\n" << systemMat << "\n";

	/*====== Programming Assignment 2 ======*/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

	// Do factorization
	if (systemMat.nonZeros() > 0) {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		} else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
}




void Deformer::deform() {
	if (mCholeskySolver == nullptr) {
		return;
	}

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques
	/* take place.
	/* Solve for the new vertex positions after the
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

  // const std::vector< Vertex* > mVertexList = mMesh->vertices();
  // int numVertex = mVertexList.size();
  // int numRoi = mRoiList.size();
	// Eigen::MatrixXd v;
	// v.resize(numVertex, 3);
  //
  // for (int i = 0; i < numRoi; ++i) {
  //   sigma(i + numVertex, 0) = mRoiList[i]->position()[0];
  //   sigma(i + numVertex, 1) = mRoiList[i]->position()[1];
  //   sigma(i + numVertex, 2) = mRoiList[i]->position()[2];
  // }
  //
  // v = mCholeskySolver->solve(AT * sigma);
  //
  // for (int i = 0; i < mVertexList.size(); ++i)
	//   mVertexList[i]->setPosition(v.row(i).transpose().cast< float >());

  const std::vector< Vertex* > mVertexList = mMesh->vertices();
  int numVertex = mVertexList.size();
  int numRoi = mRoiList.size();
	Eigen::VectorXd v;
	v.resize((numVertex + numRoi) * 3);

  for (int i = 0; i < numRoi; ++i) {
    sigma[numVertex * 3 + i] = mRoiList[i]->position()[0];
    sigma[numVertex * 3 + i + numRoi] = mRoiList[i]->position()[1];
    sigma[numVertex * 3 + i + numRoi * 2] = mRoiList[i]->position()[2];
  }

  v = mCholeskySolver->solve((AT * sigma).cast< double >());

  for (int i = 0; i < mVertexList.size(); ++i)
	  mVertexList[i]->setPosition(Eigen::Vector3f(v[i], v[i + numVertex], v[i + numVertex * 2]));

	/*====== Programming Assignment 2 ======*/
}
