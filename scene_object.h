/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    classes defining primitives in the scene

***********************************************************/

#include "util.h"
#include <float.h>

// All primitives should provide a intersection function.
// To create more primitives, inherit from SceneObject.
// Namely, you can create, Sphere, Cylinder, etc... classes
// here.
class SceneObject {
public:
  // Returns true if an intersection occured, false otherwise.
  virtual bool intersect( Ray3D&, const Matrix4x4&, const Matrix4x4& ) = 0;
};

// Example primitive you can create, this is a unit square on
// the xy-plane.
class UnitSquare : public SceneObject {
public:
  UnitSquare() {
    bound = 0.5;
  }
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld );

protected:
  double bound;
};

class Plane : public UnitSquare {
public:
  Plane() {
    bound = DBL_MAX;
  }
};

class UnitSphere : public SceneObject {
public:
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld );
};

// Defines a general quadratic surface represented by an implicit quadratic surface
class GeneralQuadratic : public SceneObject {
public:
  bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld );
  GeneralQuadratic( double a, double b, double c, double d, double e,
      double f, double g, double h, double i, double j):
      a(a), b(b), c(c), d(d), e(e), f(f), g(g), h(h), i(i), j(j) {}
private:
  double a, b, c, d, e, f, g, h, i, j;
};
