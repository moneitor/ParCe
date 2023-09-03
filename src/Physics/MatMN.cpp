#include "MatMN.h"

MatMN::MatMN()
    :M{0}, N{0}, vectors{nullptr}
{
}

MatMN::MatMN(int _rows, int _columns)
    :M{_rows},
    N{_columns}
{
    vectors = new VecN[M];
    for (int i = 0; i < M; i++)
    {
        vectors[i] = VecN(N);
    }
}

MatMN::MatMN(const MatMN &other)
{
    *this = other;
}

MatMN::~MatMN()
{
    delete[] vectors;
}

void MatMN::Zero()
{
    for (int i = 0; i < M; i++)
    {
        vectors[i].Zero();
    }
}

MatMN MatMN::Transpose() const
{
    MatMN result(N, M);

    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            result.vectors[j][i] = vectors[i][j];
        }
    }

    return result;
}

VecN MatMN::SolveGaussSeidel(const MatMN &mat, const VecN &vec)
{
    const int N = vec.N;
    VecN X = VecN(N);
    X.Zero();

    for (int iterations = 0; iterations < N; iterations++)
    {
        for (int i = 0; i < N; i++)
        {
            if (mat.vectors[i][i] != 0)
            {
                X[i] += (vec[i] / mat.vectors[i][i]) - (mat.vectors[i].Dot(X) / mat.vectors[i][i]);
            }
        }
    }
    return X;    
}

const MatMN &MatMN::operator=(const MatMN &other)
{
    this->M = other.M;
    this->N = other.N;

    delete[] vectors;
    vectors = new VecN[M];
    for (int i = 0 ; i < M; i++)
    {
        vectors[i] = other.vectors[i];
    }

    return *this;
}

VecN MatMN::operator*(const VecN &v) const
{
    if (v.N != N)
    {
        return v;
    }

    VecN result = VecN(M);

    for (int i = 0; i < M; i++)
    {
        result[i] = v.Dot(vectors[i]);
    }

    return result;
    
}

MatMN MatMN::operator*(const MatMN &m) const
{
    if (m.M != N && m.N != M)
    {
        return m;
    }

    MatMN transposed = m.Transpose();
    MatMN result = MatMN(M, m.N);

    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < m.N; j++)
        {
            result.vectors[i][j] = vectors[i].Dot(transposed.vectors[j]);
        }
    }

    return result;
}
