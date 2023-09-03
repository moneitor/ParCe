#pragma once
#include "VecN.h"

struct MatMN
{
    MatMN();
    MatMN(int _rows, int _columns);
    MatMN(const MatMN &other);
    ~MatMN();

    int M;
    int N;

    VecN *vectors;

    void Zero();
    MatMN Transpose() const;
    static VecN SolveGaussSeidel(const MatMN &mat, const VecN &vec);

    const MatMN &operator = (const MatMN &other);
    VecN operator * (const VecN &v) const;
    MatMN operator * (const MatMN &m) const;
};