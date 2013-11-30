/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include <limits>
#include "scene_object.h"
#include "render_style.h"

#define LAMBDA_EPSILON 0.000001

// Finds intersection for UnitSquare, which is
// defined on the xy-plane, with vertices (0.5, 0.5, 0),
// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
// (0, 0, 1).
bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {

  // Transform ray into object space
  Point3D modelPoint = worldToModel*ray.origin;
  Vector3D modelDirection = worldToModel*ray.dir;

  // The square's normal and a point on the square
  Vector3D normal = Vector3D(0, 0, ray.origin[2] > 0 ? 1 : -1);
  Point3D q1 = Point3D(0, 0, 0);

  // Find how close the intersection is
  double lambda = dot(q1 - modelPoint, normal)/dot(modelDirection, normal);

  // If a closer intersection exists, ignore this one
  if ((ray.intersection.t_value < lambda && !ray.intersection.none) || (lambda < 0) || (ray.startObject && ray.startObject == this && lambda < LAMBDA_EPSILON)) {
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
    ray.intersection.sceneObject = this;
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
      } else if (ld1 > ld2 && ld2 > LAMBDA_EPSILON){
        lambda = ld2;
      } else {
        lambda = ld1;
      }
    }

    if ((ray.intersection.t_value < lambda && !ray.intersection.none) || (ld1 < 0 && ld2 < 0) || (lambda < 0) || (ray.startObject && ray.startObject == this && lambda < LAMBDA_EPSILON)) {
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
    ray.intersection.sceneObject = this;
  }

  return didIntersect;
}

// reference: http://mrl.nyu.edu/~perlin/courses/fall2013/sep25b/
bool GeneralQuadratic::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
      const Matrix4x4& modelToWorld ) {

  Point3D point = worldToModel * ray.origin;
  Vector3D dir = worldToModel * ray.dir;

  double wx = dir[0];
  double wy = dir[1];
  double wz = dir[2];

  double vx = point[0];
  double vy = point[1];
  double vz = point[2];

  double A = a * wx * wx + b * wy * wy + c * wz * wz
           + d * wy * wz + e * wz * wx + f * wx * wy;
  double B = 2.0f * (a * vx * wx + b * vy * wy + c * vz * wz)
           + d * (vy * wz + vz * wy)
           + e * (vz * wx + vx * wz)
           + f * (vx * wy + vy * wx)
           + g * wx
           + h * wy
           + i * wz;
  double C = a * vx * vx + b * vy * vy + c * vz * vz
           + d * vy * vz + e * vz * vx + f * vx * vy
           + g * vx + h
            * vy + i * vz + j;
  double D = B * B - 4 * A * C;
  bool didIntersect = D >= 0;

  // Find how close the intersection is
  if (didIntersect) {
    double lambda = - B / (2.0f * A);
    double ld1, ld2;
    ld1 = ld2 = 0;
    if (D > 0) {
      ld1 = lambda + sqrt(D) / (2.0f * A);
      ld2 = lambda - sqrt(D) / (2.0f * A);
      if (ld1 > 0 && ld2 < 0) {
        lambda = ld1;
      } else if (ld1 > ld2 && ld2 > LAMBDA_EPSILON){
        lambda = ld2;
      } else {
        lambda = ld1;
      }
    }

    if ((!ray.intersection.none && ray.intersection.t_value < lambda) || (ld1 < 0 && ld2 < 0) || (lambda < 0) || (ray.startObject && ray.startObject == this && lambda < LAMBDA_EPSILON)) {
      return false;
    }

    // Find the intersection and associated normal
    Point3D intersection = point + lambda * dir;

    double x = intersection[0];
    double y = intersection[1];
    double z = intersection[2];

    Vector3D normal = Vector3D( 2.0f * a * x + e * z + f * y + g,
                                2.0f * b * y + d * z + f * x + h,
                                2.0f * c * z + d * y + e * x + i );

    // Populate ray.intersection values
    ray.intersection.none = !didIntersect;
    ray.intersection.point = modelToWorld * intersection;
    ray.intersection.t_value = lambda;
    normal = transNorm(worldToModel, normal);
    normal.normalize();
    ray.intersection.normal = normal;
    ray.intersection.sceneObject = this;
  }

  return didIntersect;
}
