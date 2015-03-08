#ifndef _MATRIX4_H_
#define _MATRIX4_H_

#define M_PI 3.14159265358979323846
#include <string>
#include "Vector3.h"
#include "Vector4.h"

class Matrix4
{
  protected:
    double m[4][4];   // matrix elements; first index is for rows, second for columns (row-major)
    
  public:
    Matrix4();
    Matrix4& operator=(const Matrix4&);
    double* getPointer(); 
    void identity(); 
    void transpose();
    void makeRotateY(double); 

	Matrix4& operator*(const Matrix4&);
	Vector4& operator*(const Vector4&);
	void makeRotateX(double);
	void makeRotateZ(double);
	void makeRotate(double, const Vector3&);
	void makeScale(double, double, double);
	void makeTranslate(double, double, double);
	void print(std::string);
	Vector3& getV3();
	void setM4cm(Vector3, Vector3, Vector3, Vector3); // manually set matrix column major V3
};

#endif