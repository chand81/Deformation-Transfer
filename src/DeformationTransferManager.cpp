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

#include <DeformationTransferManager.h>
#include <algorithm>

#define  EPSILON 0.0001

DeformationTransferManager::DeformationTransferManager()
{
	U = Uk = Ur = NULL;
	defVerts = NULL;
	bLog = false;
	regularization = 0;
	Reset();
}

DeformationTransferManager::~DeformationTransferManager()
{
	Reset();
}
void DeformationTransferManager::Reset()
{
	nTris = nVerts = 0;
	if (U != NULL)		delete U;		U = NULL;
	if (Uk != NULL)		delete Uk;	Uk = NULL;
	if (Ur != NULL)		delete Ur;	Ur = NULL;
	if (defVerts != NULL) delete defVerts; defVerts = NULL;
}

void DeformationTransferManager::SetLogging(bool bLogging)
{
	bLog = bLogging;
}

bool DeformationTransferManager::SetSourceATargetA( 
	int nTriagles, int nVertices, const int* triVertexIds, const float* vertexLocsSA, 
	const float* vertexLocsTA, ConstrainedVertIDs *vertConstrains)
{
	Reset();

	nTris = nTriagles;
	nVerts = nVertices;
	triVertIds= triVertexIds;
	vertLocsSA = vertexLocsSA;
	vertLocsTA = vertexLocsTA;
	
	if (vertConstrains == NULL || vertConstrains->size()==0) 
	{
		// we need to constrain atleast one vertex
		constrainedVertIDs.clear();
		constrainedVertIDs.push_back(0);
	}
	else
		constrainedVertIDs = *vertConstrains;

	MatrixXd Va(3,2);

	U = new SparseMatrix<double>(3*nTris,nVerts); //U->setZero();
	defVerts = new float[3*nVerts];

	int i,j;
	MatrixXd Uj(2,3);
	for (j=0; j<nTris; j++)
	{
		int iV1 = triVertIds[j*3+0];
		int iV2 = triVertIds[j*3+1];
		int iV3 = triVertIds[j*3+2];

		Vector3f v1(&vertLocsTA[iV1*3]);
		Vector3f v2(&vertLocsTA[iV2*3]);
		Vector3f v3(&vertLocsTA[iV3*3]);

		SetMatrixCol(Va, v2-v1, 0);
		SetMatrixCol(Va, v3-v1, 1);
		MatrixXd Q(3,2); MatrixXd R(2,2); R.setZero();
		QRFactorize(Va, Q, R);

		if (bLog)
		{
			std::cerr << "Matrix Va:\n" << Va << "\n\n";
			std::cerr << "Matrix Q:\n" << Q << "\n\n";
			std::cerr << "Matrix R:\n" << R << "\n\n";
		}

		R = R.inverse(); Q.transposeInPlace();
		Uj = R * Q;
		
		if (bLog) std::cerr << "Matrix Uj:\n" << Uj << "\n\n";

		(*U).coeffRef(j*3+0, iV1) = -Uj(0,0) - Uj(1,0);
		(*U).coeffRef(j*3+0, iV2) = Uj(0,0);
		(*U).coeffRef(j*3+0, iV3) = Uj(1,0);

		(*U).coeffRef(j*3+1, iV1) = -Uj(0,1) - Uj(1,1);
		(*U).coeffRef(j*3+1, iV2) = Uj(0,1);
		(*U).coeffRef(j*3+1, iV3) = Uj(1,1);

		(*U).coeffRef(j*3+2, iV1) = -Uj(0,2) - Uj(1,2);
		(*U).coeffRef(j*3+2, iV2) = Uj(0,2);
		(*U).coeffRef(j*3+2, iV3) = Uj(1,2);

	}
	if (bLog) std::cerr << "Matrix U:\n" << *U << "\n\n";

	// constrain vertices
	int nCVerts = constrainedVertIDs.size();
	std::sort(constrainedVertIDs.begin(), constrainedVertIDs.end());
	Uk = new SparseMatrix<double>(3*nTris,nVerts-nCVerts);
	Ur = new SparseMatrix<double>(3*nTris,nCVerts);
	int indxUk=0, indxUr=0, indxCVID=0, nextCVID = constrainedVertIDs[0];
	for (i=0; i<nVerts; i++)
	{
		if (i == nextCVID)
		{
			indxCVID++;
			if (indxCVID >= nCVerts)
			{
				indxCVID = -1;
			}
			Ur->col(indxUr++) = U->col(i);
			nextCVID = constrainedVertIDs[indxCVID];
			continue;
		}
		Uk->col(indxUk++) = U->col(i);
	}
	Ut = Uk->transpose();
	UtU = Ut * (*Uk);

	// Regularize matrix
	if (regularization !=0)
	{
		for (i=0; i<nVerts-1; i++)
		{
			UtU.coeffRef(i,i) += regularization;
		}
	}

	solver.compute(UtU);
	if(solver.info()!=Eigen::Success) 
	{
		// decomposition failed
		strLastError = solver.lastErrorMessage();
		return false;
	}

	return true;
}

bool DeformationTransferManager::TransferDeformation(
	const float* vertLocsSB, const float* vertLocsTB, float** defVertLocs)
{
	// Allocate and zero set matrix S.
	MatrixXd *S = new MatrixXd(3*nTris,3); S->setZero();

	MatrixXd Va(3,3);
	MatrixXd Vb(3,3);
	MatrixXd Sa(3,3);
	int i;

	for (i=0; i<nTris; i++)
	{
		SetTriEdgeMatrix(Va, nTris, triVertIds, vertLocsSA, i);
		SetTriEdgeMatrix(Vb, nTris, triVertIds, vertLocsSB, i);

		MatrixXd Q(3,3); MatrixXd R(3,3); R.setZero();
		QRFactorize(Va, Q, R);

		if (bLog)
		{
			std::cerr << "Matrix Va:\n" << Va << "\n\n";
			std::cerr << "Matrix Q:\n" << Q << "\n\n";
			std::cerr << "Matrix R:\n" << R << "\n\n";
		}

		R = R.inverse(); Q.transposeInPlace();
		Sa = Vb * R * Q;

		if (bLog)
		{
			std::cerr << "Matrix Vb:\n" << Vb << "\n\n";
			std::cerr << "Matrix Sa:\n" << Sa << "\n\n";
		}

		Sa.transposeInPlace();
		SetMatrixBlock(*S, Sa, i*3, 0);
	}
	if (bLog)	std::cerr << "Matrix S:\n" << *S << "\n\n";

	int nCVerts = constrainedVertIDs.size();
	MatrixXd Vc(nCVerts, 3);
	int indxUk=0, indxUr=0, indxCVID=0, nextCVID = constrainedVertIDs[0];
	for (i=0; i<nCVerts; i++)
	{
		Vector3f cVert(&vertLocsTB[3*constrainedVertIDs[i]]);
		Vc.row(i) = cVert.cast<double>();
	}
	*S -= (*Ur) * Vc;

	MatrixXd UtS = Ut * (*S);

	MatrixXd X = solver.solve(UtS);
	if(solver.info()!=Eigen::Success) 
	{
		// solving failed
		strLastError = solver.lastErrorMessage();
		return false;
	}

	delete S; 

	float *currVert = defVerts;

	int indxX=0;
	indxCVID=0, nextCVID = constrainedVertIDs[0];
	for (i=0; i<nVerts; i++)
	{
		if (i == nextCVID)
		{
			indxCVID++;
			if (indxCVID >= nCVerts)
			{
				indxCVID = -1;
			}
			Vector3f cVert(&vertLocsTB[3*(nextCVID)]);
			*(currVert++) = cVert(0,0);
			*(currVert++) = cVert(1,0);
			*(currVert++) = cVert(2,0);
			nextCVID = constrainedVertIDs[indxCVID];
			continue;
		}
		else
		{
			*(currVert++) = (float)X(indxX,0);
			*(currVert++) = (float)X(indxX,1);
			*(currVert++) = (float)X(indxX,2);
			indxX++;
		}
	}

	*defVertLocs = defVerts;

	return true;
}


void DeformationTransferManager::SetTriEdgeMatrix(MatrixXd &Va, int nTris, const int* triVertIds, const float* pts, int iTri)
{
	int iV1 = triVertIds[iTri*3+0];
	int iV2 = triVertIds[iTri*3+1];
	int iV3 = triVertIds[iTri*3+2];

	Vector3f v1(&pts[iV1*3]);
	Vector3f v2(&pts[iV2*3]);
	Vector3f v3(&pts[iV3*3]);

	Vector3f e1 = v2-v1;
	Vector3f e2 = v3-v1;
	Vector3f e3 = e1.cross(e2);
	float e3Len = e3.norm();
	if (e3Len > EPSILON)
		e3 /= e3Len;

	SetMatrixCol(Va, e1, 0);
	SetMatrixCol(Va, e2, 1);
	SetMatrixCol(Va, e3, 2);
}

void DeformationTransferManager::SetMatrixCol(MatrixXd &m, Vector3f v, int iCol)
{
	m(0,iCol) = v(0,0);
	m(1,iCol) = v(1,0);
	m(2,iCol) = v(2,0);
}

void DeformationTransferManager::SetMatrixBlock(MatrixXd &mBig, MatrixXd &mSmall, int iRow, int iCol)
{
	int r,c, rmax=mSmall.rows(), cmax=mSmall.cols();
	for (r=0; r<rmax; r++)
	{
		for (c=0; c<cmax; c++)
		{
			mBig(iRow+r, iCol+c) = mSmall(r,c);
		}
	}
}

void DeformationTransferManager::SetSparseMatrixBlock(SparseMatrix<double> &mBig, MatrixXd &mSmall, int iRow, int iCol)
{
	int r,c, rmax=mSmall.rows(), cmax=mSmall.cols();
	for (r=0; r<rmax; r++)
	{
		for (c=0; c<cmax; c++)
		{
			mBig.coeffRef(iRow+r, iCol+c) = mSmall(r,c);
		}
	}
}

void DeformationTransferManager::QRFactorize(const MatrixXd &a, MatrixXd &q, MatrixXd &r)
{


	int i,j, imax, jmax;
	imax = a.rows();
	jmax = a.cols();

	for (j=0; j<jmax; j++)
	{
		Eigen::VectorXd v (a.col(j));
		for (i=0; i<j; i++)
		{
			Eigen::VectorXd qi (q.col(i));
			r(i,j)= qi.dot(v);
			v = v - r(i,j)*qi;
		}
		float vv = (float)v.squaredNorm();
		float vLen = sqrtf(vv);
		if (vLen < EPSILON)
		{
			r(j,j) = 1;
			q.col(j).setZero();
		}
		else
		{
			r(j,j) = vLen;
			q.col(j) = v/vLen;
		}
	}
}

std::string DeformationTransferManager::GetLastError()
{
	return strLastError;
}

void DeformationTransferManager::SetRegularization( float regVal )
{
	regularization = regVal;
}

