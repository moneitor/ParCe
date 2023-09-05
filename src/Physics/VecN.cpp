#include "VecN.h"

VecN::VecN()
    :N{0},
    data{nullptr}
{
}

VecN::VecN(int _N)
    :N{_N}
{
    data = new float[_N];
}

VecN::VecN(const VecN &other)
{
    this->N = other.N;
    this->data = new float[N];
    for (int i = 0; i < N; i++)
    {
        this->data[i] = other.data[i];
    }
}

VecN::~VecN()
{
    delete[] data;
}

VecN &VecN::operator=(const VecN &other)
{
    if(this == &other)
    {
        return *this;
    }

    if (data) 
    {
        delete[] data;
    }

    N = other.N;

    data = new float [N];
    for(int i = 0; i < N; i++)
    {
        this->data[i] = other.data[i];
    }

    return *this;
}

VecN VecN::operator+(const VecN &other)
{

    assert(this->N != other.N &&  "Vectors must be same size");

    VecN newVec = *this;
    for (int i = 0; i < N; i++)
    {
        newVec.data[i] += other.data[i];
    }
    return newVec;
}

VecN VecN::operator-(const VecN &other)
{
    assert(this->N != other.N &&  "Vectors must be same size");

    VecN newVec = *this;
    for (int i = 0; i < N; i++)
    {
        newVec.data[i] -= other.data[i];
    }
    return newVec;
}

VecN VecN::operator*(const float scalar)
{
    VecN newVec = *this;
    newVec *= scalar;
    return newVec;    
}

const VecN &VecN::operator+=(const VecN &other)
{
    // assert(this->N != other.N &&  "Vectors must be same size");

    for (int i = 0; i < N; i++)
		data[i] += other.data[i];
	return *this;
}

const VecN &VecN::operator-=(const VecN &other)
{    
    assert(this->N != other.N &&  "Vectors must be same size");

    for (int i = 0; i < N; i++)
		data[i] -= other.data[i];
	return *this;
}

const VecN &VecN::operator*=(const float scalar)
{
    for (int i = 0; i < N; i++)
		data[i] *= scalar;
	return *this;
}

float VecN::operator[](const int index) const
{
    return this->data[index];
}

float &VecN::operator[](const int index)
{
    return this->data[index];
}

float VecN::Dot(const VecN &v) const
{
    float dot = 0.0f;
    for (int i = 0; i < N; i++)
    {
        dot += this->data[i] * v.data[i];
    }
    return dot;
}

void VecN::Zero()
{
    for (int i = 0; i < N; i++)
    {
        this->data[i] = 0;
    }
}
