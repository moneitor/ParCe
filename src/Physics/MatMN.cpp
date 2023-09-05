#include "MatMN.h"

MatMN::MatMN()
    : M(0), 
    N(0), 
    vectors(nullptr) 
{

}

MatMN::MatMN(int M, int N)
    : M(M), 
    N(N) 
{
	vectors = new VecN[M];
	for (int i = 0; i < M; i++)
		vectors[i] = VecN(N);
}

MatMN::MatMN(const MatMN& m) 
{
    *this = m;
}

MatMN::~MatMN() 
{
    delete[] vectors;
}

void MatMN::Zero() 
{
	for (int i = 0; i < M; i++)
		vectors[i].Zero();
}

MatMN MatMN::Transpose() const 
{
    MatMN result(N, M);
	for (int i = 0; i < M; i++)
		for (int j = 0; j < N; j++)
			result.vectors[j][i] = vectors[i][j];
	return result;
}

const MatMN& MatMN::operator = (const MatMN& m) 
{
	M = m.M;
	N = m.N;
	vectors = new VecN[M];
	for (int i = 0; i < M; i++)
		vectors[i] = m.vectors[i];
	return *this;
}

VecN MatMN::operator * (const VecN& v) const 
{
	if (v.N != N)
		return v;
	VecN result(M);
	for (int i = 0; i < M; i++)
		result[i] = v.Dot(vectors[i]);
	return result;
}

MatMN MatMN::operator * (const MatMN& m) const 
{
	if (m.M != N && m.N != M)
		return m;		
	MatMN tranposed = m.Transpose();
	MatMN result(M, m.N);
	for (int i = 0; i < M; i++)
		for (int j = 0; j < m.N; j++)
			result.vectors[i][j] = vectors[i].Dot(tranposed.vectors[j]);
	return result;
}

VecN MatMN::SolveGaussSeidel(const MatMN& A, const VecN& b) 
{
	const int N = b.N;
	VecN X(N);
	X.Zero();

	// Iterate N times
	for (int iterations = 0; iterations < N; iterations++) 
    {
		for (int i = 0; i < N; i++) 
        {
			if (A.vectors[i][i] != 0.0f)
            {
				X[i] += (b[i] / A.vectors[i][i]) - (A.vectors[i].Dot(X) / A.vectors[i][i]);
			}
		}
	}
	return X;
}
