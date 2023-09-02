#pragma once

#include <iostream>

struct VecN
{   
    VecN();
    VecN(int _N);
    VecN(const VecN &other);

    ~VecN();

    //  + - * += -= *= v[*] dot(a, b)  

    VecN &operator = (const VecN &other);
    VecN operator + (const VecN &other); // v1 + v2
    VecN operator - (const VecN &other); // v1 - v2
    VecN operator * (const float scalar); // v1 * v2
    const VecN &operator += (const VecN &other); // v1 += v2
    const VecN &operator -= (const VecN &other); // v1 -= v2
    const VecN &operator *= (const float scalar); // v1 *= v2
    float operator [] (const int index) const;  // v1[index]
    float &operator [] (const int index);  // v1[index] = value

    float Dot(const VecN v) const;
    void Zero();

    int N;
    float *data;
};