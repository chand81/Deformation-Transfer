/*	
	The MIT License (MIT)

	Copyright (c) 2014 Chandan Pawaskar (chandan.pawaskar@gmail.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <Eigen/SparseQR>

using Eigen::DynamicSparseMatrix;
using Eigen::SparseMatrix;
using Eigen::MatrixXd;
using Eigen::HouseholderQR;
using Eigen::Vector3d;
using Eigen::Vector3f;

typedef std::vector<unsigned int> ConstrainedVertIDs;

class DeformationTransferManager
{
public:

	DeformationTransferManager();
	~DeformationTransferManager();

	//************************************
	// Method:    Set the SourceA and TargetA meshes
	// Returns:   True if successful else false
	// Parameter: int nTriangles - number of triangles
	// Parameter: int nVertices - number of vertices
	// Parameter: const int * triVerterxIds - pointer to triangle vertex id's for SourceA mesh, has length = nTriangles * 3
	// Parameter: const float * vertexLocsSA - pointer to vertex locations for SourceA mesh, has length = nVertices * 3
	// Parameter: const float * vertexLocsTA - pointer to vertex locations for TargetA mesh, has length = nVertices * 3
	//************************************
	bool SetSourceATargetA(int nTriangles, int nVertices, const int* triVertexIds, const float* vertexLocsSA, 
		const float* VertexLocsTA, ConstrainedVertIDs *vertConstrains = NULL);
	
	//************************************
	// Method:    Transfers deformation of SourceA-SourceB to TargetA and returns pointer to resulting vertex locations in defVertLocs
	// Returns:   True if succeeded else false.
	// Parameter: const float * vertLocsSB - pointer to vertex locations for SourceB mesh, has length = nVertices * 3
	// Parameter: const float * vertLocsTB - pointer to vertex locations for TargetB mesh, has length = nVertices * 3
	// Parameter: float * * defVertLocs - pointer to a float pointer. This is updated to the location where the result is stored.
	//************************************
	bool TransferDeformation(const float* vertLocsSB, const float* vertLocsTB, float** defVertLocs);

	//************************************
	// Method:    Query the last error string.
	// Returns:   std::string 
	//************************************
	std::string GetLastError();

	//************************************
	// Method:    Set to true to enable logging. Default is disabled.
	// Parameter: bool bLogging 
	//************************************
	void SetLogging(bool bLogging);


	//************************************
	// Method:    Set Regularization value to be used. This can help
	//						stabilize numerically unstable matrices without affecting 
	//						desirable results much.
	// Parameter: float regVal Set to zero or negative to disable.
	//************************************
	void SetRegularization(float regVal);

protected:

private:

	int nTris, nVerts;
	const int *triVertIds;
	const float *vertLocsSA, *vertLocsTA;
	float *defVerts;

	SparseMatrix<double> *U, *Uk, *Ur, Ut, UtU;

	std::string strLastError;

	bool bLog; float regularization;

	ConstrainedVertIDs constrainedVertIDs;
	Eigen::SparseLU< SparseMatrix<double> > solver;

	void SetMatrixBlock(MatrixXd &mBig, MatrixXd &mSmall, int iRow, int iCol);
	void SetSparseMatrixBlock(SparseMatrix<double> &mBig, MatrixXd &mSmall, int iRow, int iCol);
	void SetMatrixCol(MatrixXd &m, Vector3f v, int iCol);
	void SetTriEdgeMatrix(MatrixXd &Va, int nTris, const int* triVertIds, const float* pts, int iTri);
	void Reset();


	//************************************
	// Method:    q,r should be preallocated and r must be cleared.
	// Parameter: const MatrixXd & a
	// Parameter: MatrixXd & q
	// Parameter: MatrixXd & r
	//************************************
	void QRFactorize(const MatrixXd &a, MatrixXd &q, MatrixXd &r);
};