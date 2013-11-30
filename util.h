/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    utility functions and structures
    (based on code from CGL, University of Waterloo),
    modify this file as you see fit.

***********************************************************/

#ifndef _UTIL_
#define _UTIL_

#include <iostream>
#include <cmath>
#include <cstdlib>

class SceneObject;

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#define RANDOM ((double) rand() / (RAND_MAX))

class Point3D {
public:
  Point3D();
  Point3D(double x, double y, double z);
  Point3D(const Point3D& other);

  Point3D& operator =(const Point3D& other);
  double& operator[](int i);
  double operator[](int i) const;

private:
  double m_data[3];
};

class Vector3D {
public:
  Vector3D();
  Vector3D(double x, double y, double z);
  Vector3D(const Vector3D& other);

  Vector3D& operator =(const Vector3D& other);
  double& operator[](int i);
  double operator[](int i) const;

  double length() const;
  double normalize();
  double dot(const Vector3D& other) const;
  Vector3D cross(const Vector3D& other) const;

private:
  double m_data[3];
};

// standard operators on points and vectors
Vector3D operator *(double s, const Vector3D& v);
Vector3D operator +(const Vector3D& u, const Vector3D& v);
Point3D operator +(const Point3D& u, const Vector3D& v);
Vector3D operator -(const Point3D& u, const Point3D& v);
Vector3D operator -(const Vector3D& u, const Vector3D& v);
Vector3D operator -(const Vector3D& u);
Point3D operator -(const Point3D& u, const Vector3D& v);
Vector3D cross(const Vector3D& u, const Vector3D& v);
double dot(const Vector3D& u, const Vector3D& v);
double dot(const Point3D& u, const Point3D& v);
double dot(const Point3D& u, const Vector3D& v);
std::ostream& operator <<(std::ostream& o, const Point3D& p);
std::ostream& operator <<(std::ostream& o, const Vector3D& v);

Vector3D orthogonal(const Vector3D& u);
Vector3D orthogonal(const Vector3D& u, const Vector3D& v);
Vector3D randomDeviation(const Vector3D& dir, const Vector3D& planeU, const Vector3D& planeV, double randAmount);

class Vector4D {
public:
  Vector4D();
  Vector4D(double w, double x, double y, double z);
  Vector4D(const Vector4D& other);

  Vector4D& operator =(const Vector4D& other);
  double& operator[](int i);
  double operator[](int i) const;

private:
  double m_data[4];
};

class Matrix4x4 {
public:
  Matrix4x4();
  Matrix4x4(const Matrix4x4& other);
  Matrix4x4& operator=(const Matrix4x4& other);

  Vector4D getRow(int row) const;
  double *getRow(int row);
  Vector4D getColumn(int col) const;

  Vector4D operator[](int row) const;
  double *operator[](int row);

  Matrix4x4 transpose() const;

private:
  double m_data[16];
};

Matrix4x4 operator *(const Matrix4x4& M, const Matrix4x4& N);
Vector3D operator *(const Matrix4x4& M, const Vector3D& v);
Point3D operator *(const Matrix4x4& M, const Point3D& p);
// Multiply n by the transpose of M, useful for transforming normals.
// Recall that normals should be transformed by the inverse transpose
// of the matrix.
Vector3D transNorm(const Matrix4x4& M, const Vector3D& n);
std::ostream& operator <<(std::ostream& os, const Matrix4x4& M);

class Colour {
public:
  Colour();
  Colour(double r, double g, double b);
  Colour(const Colour& other);

  Colour& operator =(const Colour& other);
  Colour operator *(const Colour& other);
  double& operator[](int i);
  double operator[](int i) const;

  void clamp();

private:
  double m_data[3];
};

Colour operator *(double s, const Colour& c);
Colour operator /(const Colour& c, double s);
Colour &operator /=(Colour& c, double s);
Colour operator +(const Colour& u, const Colour& v);
Colour &operator +=(Colour& u, const Colour& v);
Colour operator -(const Colour& u, const Colour& v);
bool operator >(const Colour& u, double d);
bool colourDiff(const Colour& u, const Colour&v, double threshold);
std::ostream& operator <<(std::ostream& o, const Colour& c);

struct Material {
  Material( Colour ambient, Colour diffuse, Colour specular, double exp, double reflection, double ref_damping, double n, double glossiness) :
    ambient(ambient), diffuse(diffuse), specular(specular), specular_exp(exp),
    reflection(reflection), ref_damping(ref_damping), n(n), glossiness(glossiness) {}
  // Ambient components for Phong shading.
  Colour ambient;
  // Diffuse components for Phong shading.
  Colour diffuse;
  // Specular components for Phong shading.
  Colour specular;
  // Specular expoent.
  double specular_exp;
  // Fraction of secondary illumination that is reﬂected by the surface at intersection opint
  double reflection;
  // The reflactive damping coefficient; less specular reflection farther away.
  double ref_damping;
  // Speed of light in this medium. Used for refraction calculations. 0 means not refractive at all.
  double n;
   // How glossy the surface is
  double glossiness;
};

struct Intersection {
  Intersection() {
    sceneObject = NULL;
  }
  // Location of intersection.
  Point3D point;
  // Normal at the intersection.
  Vector3D normal;
  // Material at the intersection.
  Material* mat;
  // Position of the intersection point on your ray.
  // (i.e. point = ray.origin + t_value * ray.dir)
  // This is used when you need to intersect multiply objects and
  // only want to keep the nearest intersection.
  double t_value;
  // Set to true when no intersection has occured.
  bool none;
  // This points to the object with which the ray colided
  SceneObject* sceneObject;
};

// Ray structure.
struct Ray3D {
  static const int MAX_REFLECTION = 5;
  Ray3D() {
    intersection.none = true;
    reflectionNumber = 0;
    refractionNumber = 0;
    currentMedium = NULL;
    currentMaterial = NULL;
  }
  Ray3D( Point3D p, Vector3D v ) : origin(p), dir(v) {
    intersection.none = true;
    reflectionNumber = 0;
    refractionNumber = 0;
    currentMedium = NULL;
    currentMaterial = NULL;
  }
  Ray3D(const Ray3D& other);
  // Origin and direction of the ray.
  Point3D origin;
  Vector3D dir;
  // Intersection status, should be computed by the intersection
  // function.
  Intersection intersection;
  // Current colour of the ray, should be computed by the shading
  // function.
  Colour col;
  // This integer determines if the ray is a reflection or an "original" ray.
  // An original ray corresponds to a reflectioNumber of 0. This is used to
  // avoid infinite loops when doing reflections. This value must be capped
  // to max_reflection from raytracer by the programmer.
  int reflectionNumber;
  // Same this as reflectionNumber but for refraction
  int refractionNumber;
  // This points to the object where the ray originated or nothing if it originated at the light
  SceneObject* startObject;
  // This points to the object in which the ray is currently travelling. This is used for refraction. If this is NULL, it refers to the vaccum.
  SceneObject* currentMedium;
  Material* currentMaterial;
};
#endif
