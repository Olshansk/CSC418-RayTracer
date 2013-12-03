/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"
#include "render_style.h"

#define GLOSS_LIGHT_RAY_MULTIPLIER 10

void PointLight::shade( Ray3D& ray, int glossy_rays ) {
  Intersection intersection = ray.intersection;
  Material* mat = intersection.mat;
  Vector3D normal = intersection.normal;
  Vector3D incident_vec = ray.dir;

  // Since light shading doesn't spawn new rays, we can take the liberty to use more rays
  glossy_rays = glossy_rays * GLOSS_LIGHT_RAY_MULTIPLIER;

  Vector3D s_vec = get_position() - intersection.point;
  s_vec.normalize();

  Vector3D r_vec = 2.0 * (normal.dot(s_vec)) * normal - s_vec;

  Colour col = ray.col;
  col += fmax(0, normal.dot(s_vec)) * (mat->diffuse * _col_diffuse);
  if (RenderStyle::rstyle != AMBIENT_DIFFUSE) {
    if (glossy_rays != 0 && mat->glossiness != 0) {
      Vector3D ortho1 = orthogonal(r_vec);
      Vector3D ortho2 = orthogonal(r_vec, ortho1);

      Colour specCol = Colour();

      for (int i = 0; i < glossy_rays; i++) {
        Vector3D rand_r_vec = randomDeviation(r_vec, ortho1, ortho2, mat->glossiness);
        specCol += pow(fmax(0, -rand_r_vec.dot(incident_vec)), mat->specular_exp) * (mat->specular * _col_specular);
      }

      col += specCol / glossy_rays;
    } else {
      col += pow(fmax(0, -r_vec.dot(incident_vec)), mat->specular_exp) * (mat->specular * _col_specular);
    }
  }

  col.clamp();
  ray.col = col;
}

Colour PointLight::shadeAmbient(Material* mat) {
  return mat->ambient * _col_ambient;
}

Ray3D PointLight::getShadowRay( Ray3D& ray ) {
  Point3D intersectionPoint = ray.intersection.point;
  Vector3D direction = get_position() - intersectionPoint;
  Ray3D shadowRay = Ray3D(intersectionPoint, direction);
  shadowRay.sceneObject = ray.intersection.sceneObject;
  return shadowRay;
}

Ray3D LightSource::getReflectionRay( Ray3D& ray ) {
  Point3D origin = ray.intersection.point;
  Vector3D direction = 2*((-ray.dir).dot(ray.intersection.normal))*ray.intersection.normal + ray.dir;
  Ray3D reflectionRay = Ray3D(origin, direction);
  reflectionRay.reflectionNumber += 1;
  reflectionRay.sceneObject = ray.intersection.sceneObject;
  return reflectionRay;
}
