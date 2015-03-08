#include "Vector4.h"
#include <iostream>

Vector4::Vector4() { this->x = this->y = this->z = this->w = 0; }

Vector4::Vector4(double x, double y, double z, double w) {
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

Vector4::~Vector4(){}

Vector4 Vector4::operator+(const Vector4& v) {
	Vector4 *v1 = new Vector4(this->x + v.x, this->y + v.y, this->z + v.z, this->w + v.w);
	return *v1;
}

Vector4 Vector4::operator-(const Vector4& v) {
	Vector4 *v1 = new Vector4(this->x - v.x, this->y - v.y, this->z - v.z, this->w - v.w);
	return *v1;
}

void Vector4::dehomogenize() {
	x = x / w;
	y = y / w;
	z = z / w;
	w = 1.0;
}

void Vector4::print(std::string s) {
	//printf("%s \n %d %d %d %d \n", s, x, y, z, w);
	std::cout << s << std::endl << x << " " << y << " " << z << " " << w << std::endl;
}

double Vector4::getX() const {
	return this->x;
}

double Vector4::getY() const {
	return this->y;
}

double Vector4::getZ() const {
	return this->z;
}

double Vector4::getW() const {
	return this->w;
}