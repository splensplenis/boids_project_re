#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <cassert>

// using Coord = double; //type alias
class Vector {
  double x_{};
  double y_{};

 public:
  Vector();
  Vector(double, double);  // perché due tipi?
  // Vector(Vector); by deafult?
  // Vector(-Vector); should define -Vector?
  void print();
  double x() const;
  double y() const;
  Vector& operator+=(Vector const&);
  Vector& operator*=(double);
  Vector& operator-=(Vector const&);  // non usato
  Vector& operator/=(double);
};

Vector operator+(Vector const&, Vector const&);
double operator*(Vector const&, Vector const&);
Vector operator*(Vector const&, double);
Vector operator-(Vector const&, Vector const&);
Vector operator/(Vector const&, double);
bool operator==(Vector const&, Vector const&);
bool operator!=(Vector const&, Vector const&);
double norm2(Vector const&);

#endif