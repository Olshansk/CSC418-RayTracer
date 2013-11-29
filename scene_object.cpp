/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"
#include "render_style.h"

// Finds intersection for UnitSquare, which is
// defined on the xy-plane, with vertices (0.5, 0.5, 0),
// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
// (0, 0, 1).
bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {

  // Half oof the unit square's edge length
  double bound = 0.5;

  // Transform ray into object space
  Point3D modelPoint = worldToModel*ray.origin;
  Vector3D modelDirection = worldToModel*ray.dir;

  // The square's normal and a point on the square
  Vector3D normal = Vector3D(0, 0, 1);
  Point3D q1 = Point3D(0, 0, 0);

  // Find how close the intersection is
  double lambda = dot(q1 - modelPoint, normal)/dot(modelDirection, normal);

  // If a closer intersection exists, ignore this one
  if (ray.intersection.t_value < lambda && !ray.intersection.none) {
    return false;
  }

  // Find the intersection
  Point3D intersection = modelPoint + lambda*modelDirection;
  bool intersectionInBounds = intersection[0] >= -bound && intersection[0] <= bound && intersection[1] >= -bound && intersection[1] <= bound;

  // Populate ray.intersection values
  if (intersectionInBounds) {
    ray.intersection.point = modelToWorld*intersection;
    ray.intersection.normal = transNorm(worldToModel, normal);
    ray.intersection.normal.normalize();
    ray.intersection.t_value = lambda;
    ray.intersection.none = false;
  }

  return intersectionInBounds;
}

// Intersection code for UnitSphere, which is centred on the origin.
bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {

  // The sphere's radius
  double radius = 1;

  // Transform ray into object space
  Point3D modelPoint = worldToModel*ray.origin;
  Vector3D modelDirection = worldToModel*ray.dir;

  // Detemine if there is an intersection
  double a = dot(modelDirection, modelDirection);
  double b = dot(modelPoint, modelDirection);
  double c = dot(modelPoint, modelPoint) - radius;
  double d = b * b - a * c;
  double didIntersect = d >= 0;

  // Find how close the intersection is
  if (d >= 0) {
    double lambda = - b / a;
    double ld1, ld2;
    ld1 = ld2 = 0;
    if (d > 0) {
      ld1 = lambda + sqrt(d)/a;
      ld2 = lambda - sqrt(d)/a;
      if (ld1 > 0 && ld2 < 0) {
        lambda = ld1;
      } else if (ld1 > ld2 && ld2 > 0){
        lambda = ld2;
      } else {
        lambda = ld1;
      }
    }

    // If a closer intersection exists, ignore this one
    if ((ray.intersection.t_value < lambda && !ray.intersection.none) || (ld1 < 0 && ld2 < 0)) {
      return false;
    }

    // Find the intersection and associated normal
    Point3D intersection = modelPoint + lambda*modelDirection;
    Vector3D normal = Vector3D(2 * intersection[0], 2 * intersection[1], 2 * intersection[2]);

    // Populate ray.intersection values
    ray.intersection.none = !didIntersect;
    ray.intersection.point = modelToWorld*intersection;
    ray.intersection.t_value = lambda;
    normal = transNorm(worldToModel, normal);
    normal.normalize();
    ray.intersection.normal = normal;
  }

  return didIntersect;
}
