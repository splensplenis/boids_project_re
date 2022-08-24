#ifndef VECTOR_HPP
#define VECTOR_HPP


class Vector {
  double x_ = 0.;
  double y_ = 0.;

 public:
  Vector();
  Vector(double, double);
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