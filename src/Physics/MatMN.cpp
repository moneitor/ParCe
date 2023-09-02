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
