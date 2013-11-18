/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray ) {

  Vector3D* s = new Vector3D(1, 0, 0);
  Vector3D* de = new Vector3D(1, 0, 0);

  Intersection intersection = ray.intersection;

  Vector3D normal = intersection.normal;
  Material mat = *intersection.mat;

  Colour ambient = mat.ambient;
  Colour diffuse = mat.diffuse;
  Colour specular = mat.specular;
  double specular_exp = mat.specular_exp;

  double r = 0;
  double g = 0;
  double b = 0;

  //diffuse (whats the vector from light source?)
  double temp = fmax(0, dot(normal, *s));
  r += ambient[0] * temp;
  g += ambient[1] * temp;
  b += ambient[2] * temp;
  //specular
  Vector3D m = 2*dot(normal, *s)*normal - *s;
  temp = pow(fmax(0, dot(m, *de)), specular_exp);
  r += diffuse[0] * temp;
  g += diffuse[1] * temp;
  b += diffuse[2] * temp;
  //ambient
  r += specular[0];
  g += specular[1];
  b += specular[2];

  r = fmax(r, 255);
  g = fmax(g, 255);
  b = fmax(b, 255);

  Colour *colour = new Colour(r, g, b);

  //ray.col = *colour;
  // TODO: implement this function to fill in values for ray.col
  // using phong shading.  Make sure your vectors are normalized, and
  // clamp colour values to 1.0.
  //
  // It is assumed at this point that the intersection information in ray
  // is available.  So be sure that traverseScene() is called on the ray
  // before this function.

}
