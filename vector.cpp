#include "vector.hpp"

#include <cmath>
#include <iostream>

Vector::Vector() : x_{}, y_{} {}
Vector::Vector(double x = 0., double y = 0.)
: x_{x}, y_{y} {}

double Vector::x() const { return x_; }
double Vector::y() const { return y_; }

void Vector::print() {
    std::cout << "(" << x_ << "," << y_ << ")" <<'\n'; }

Vector& Vector::operator+=(Vector const& other) {
  x_ += other.x_;
  y_ += other.y_;
  return *this;
}
Vector operator+(Vector const& v1, Vector const& v2) {
  return Vector{v1.x() + v2.x(), v1.y() + v2.y()};
}
Vector& Vector::operator*=(double scalar) {
  x_ *= scalar;
  y_ *= scalar;
  return *this;
}
double operator*(Vector const& v1, Vector const& v2) {
  return {v1.x() * v2.x() + v1.y() * v2.y()};
}
Vector operator*(Vector const& v1, double scalar) {
  Vector v{v1};
  return v *= scalar;
}
Vector& Vector::operator-=(Vector const& other) {
  x_ -= other.x_;
  y_ -= other.y_;
  return *this;
}
Vector operator-(Vector const& v1, Vector const& v2) {
  return Vector{v1.x() - v2.x(), v1.y() - v2.y()};
}
Vector& Vector::operator/=(double scalar) {
  if (scalar == 0) throw std::runtime_error{"Impossible to divide by zero"};
  x_ /= scalar;
  y_ /= scalar;
  return *this;
}
Vector operator/(Vector const& v, double scalar) {
  if (scalar == 0) throw std::runtime_error{"Impossible to divide by zero1"};
  return Vector{v.x() / scalar, v.y() / scalar};
}
double norm2(Vector const& v) { return (v.x() * v.x() + v.y() * v.y()); }
bool operator==(Vector const& v1, Vector const& v2) {
  return (v1.x() == v2.x() && v1.y() == v2.y());
}
bool operator!=(Vector const& v1, Vector const& v2) { return !(v1 == v2); }
