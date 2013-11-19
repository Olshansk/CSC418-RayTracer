/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"
#include "render_style.h"

void PointLight::shade( Ray3D& ray ) {
  Intersection intersection = ray.intersection;
  Material* mat = intersection.mat;
  Vector3D normal = intersection.normal;
  Vector3D incident_vec = ray.dir;

  Vector3D s_vec = get_position() - intersection.point;
  s_vec.normalize();

  Vector3D r_vec = incident_vec - 2.0 * (normal.dot(incident_vec)) * normal;

  Colour col = ray.col;
  col = col + (mat->ambient * _col_ambient);
  col = col + fmax(0, normal.dot(s_vec)) * (mat->diffuse * _col_diffuse);
  if (RenderStyle::rstyle != AMBIENT_DIFFUSE) {
    col = col + pow(fmax(0, -r_vec.dot(incident_vec)), mat->specular_exp) * (mat->specular * _col_specular);
  }

  col.clamp();
  ray.col = col;
}
