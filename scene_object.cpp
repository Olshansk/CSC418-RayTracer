/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  double bound = 0.5;

  // HINT: Remember to first transform the ray into object space  
  // to simplify the intersection test.
  Point3D modelPoint = worldToModel*ray.origin;
  Vector3D modelDirection = worldToModel*ray.dir;
  
  // TODO: implement intersection code for UnitSquare, which is
  // defined on the xy-plane, with vertices (0.5, 0.5, 0), 
  // (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
  // (0, 0, 1).
  Vector3D* normal = new Vector3D(0, 0, 1);
  Point3D* q1 = new Point3D(bound, bound, 0);
  // Point3D* q2 = new Point3D(-bound, bound, 0);
  // Point3D* q3 = new Point3D(-bound, -bound, 0);
  // Point3D* q4 = new Point3D(bound, -bound, 0);

  double lambda = dot(*q1 - ray.origin, *normal)/dot(ray.dir, *normal);
  
  // Your goal here is to fill ray.intersection with correct values
  // should an intersection occur.  This includes intersection.point, 
  // intersection.normal, intersection.none, intersection.t_value. 

  Point3D intersection = modelPoint + lambda*modelDirection;

  bool intersectionInBounds = intersection[0] >= -bound && intersection[0] <= bound && intersection[1] >= -bound && intersection[1] <= bound;
  
  ray.intersection.none = !intersectionInBounds;
  
  if (intersectionInBounds) {
    ray.intersection.point = modelToWorld*intersection;
    ray.intersection.normal = modelToWorld*(*normal);
    ray.intersection.t_value = lambda;
  }

  return intersectionInBounds;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  // HINT: Remember to first transform the ray into object space  
  // to simplify the intersection test.
  Point3D modelPoint = worldToModel*ray.origin;
  Vector3D modelDirection = worldToModel*ray.dir;

  // TODO: implement intersection code for UnitSphere, which is centred 
  // on the origin.  
  double radius = 1;
  double a = dot(ray.dir, ray.dir);
  double b = dot(ray.origin, ray.dir);
  double c = dot(ray.origin, ray.origin) - radius;
  double d = b * b - a * c;
  
  double didIntersect = d >= 0;
  ray.intersection.none = !didIntersect;

  // Your goal here is to fill ray.intersection with correct values
  // should an intersection occur.  This includes intersection.point, 
  // intersection.normal, intersection.none, intersection.t_value.   
  if (d>= 0) {
    double lambda = - b / a;
    if (d > 0) {
      double ld1 = - b / a + sqrt(d)/a;
      double ld2 = - b / a - sqrt(d)/a;
      if (ld1 > ld2) {
        lambda = ld1;
      } else {
        lambda = ld2;
      }
    }

    Point3D intersection = modelPoint + lambda*modelDirection;
    ray.intersection.point = modelToWorld*intersection;
    ray.intersection.t_value = lambda;
    Vector3D* normal = new Vector3D(2 * intersection[0], 2 * intersection[1], 2 * intersection[2]);
    normal->normalize();
    ray.intersection.normal = *normal;
  } 
  
  return didIntersect;
}