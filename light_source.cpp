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

void determineCurrentMedium( Ray3D& newRay, Ray3D& oldRay ) {
  if (oldRay.currentMedium == oldRay.intersection.sceneObject) {
    newRay.currentMedium = NULL;
    newRay.currentMaterial = NULL;
  } else {
    newRay.currentMedium = oldRay.intersection.sceneObject;
    newRay.currentMaterial = oldRay.intersection.mat;
  }
}

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
    if (glossy_rays != 0 && mat->glossiness > UNUSED_MATERIAL_PROPERTY_VALUE) {
      Vector3D ortho1 = orthogonal(r_vec);
      Vector3D ortho2 = orthogonal(r_vec, ortho1);

      Colour specCol = Colour();

      for (int i = 0; i < glossy_rays; i++) {
        Vector3D rand_r_vec = randomDeviation(r_vec, ortho1, ortho2, mat->glossiness);
        specCol += shadeAmbient(r_vec, incident_vec, mat);
      }

      col += specCol / glossy_rays;
    } else {
      col += shadeAmbient(r_vec, incident_vec, mat);
    }
  }

  col.clamp();
  ray.col = col;
}

Colour PointLight::shadeAmbient(Vector3D r_vec, Vector3D incident_vec, Material* mat) {
  return pow(fmax(0, -r_vec.dot(incident_vec)), mat->specular_exp) * (mat->specular * _col_specular);
}

Colour PointLight::shadeAmbient(Material* mat) {
  return mat->ambient * _col_ambient;
}

Ray3D PointLight::getShadowRay( Ray3D& ray ) {
  Point3D intersectionPoint = ray.intersection.point;
  Vector3D direction = get_position() - intersectionPoint;
  Ray3D shadowRay = Ray3D(intersectionPoint, direction);
  shadowRay.startObject = ray.intersection.sceneObject;
  return shadowRay;
}

Ray3D LightSource::getReflectionRay( Ray3D& ray ) {
  Vector3D dir = ray.dir;
  Vector3D normal = ray.intersection.normal;
  Vector3D direction = 2*((-dir).dot(normal)) * normal + dir;
  Ray3D reflectionRay = Ray3D(ray.intersection.point, direction);
  reflectionRay.reflectionNumber = ray.reflectionNumber + 1;
  reflectionRay.startObject = ray.intersection.sceneObject;
  return reflectionRay;
}

std::pair <Ray3D,double> LightSource::getRefractionRay( Ray3D& ray ) {

  Vector3D normal = ray.intersection.normal;
  Vector3D dir = -ray.dir;
  double n1 = 1.0f;
  double n2 = ray.intersection.mat->n;
  bool isExiting = !(normal.dot(ray.dir) < 0);

  if (isExiting) {
    normal = -normal;
    if (ray.currentMedium != NULL) {
      n1 = ray.currentMaterial->n;
      n2 = 1.0;
    } else {
      return std::make_pair (Ray3D(Point3D(0,0,0), Vector3D(0,0,0)),0);
    }
  }

  double n = n1 / n2;

  double cosI = normal.dot(dir);
  double sinT = n * n * (1.0 - cosI * cosI);
  float cosT = sqrtf(1.0 - sinT);

  if(cosT < 0.0){
    return std::make_pair (Ray3D(Point3D(0,0,0), Vector3D(0,0,0)),0);
  }

  double r0rth = (n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT);
  double rPar = (n2 * cosI - n1 * cosT) / (n2 * cosI + n1 * cosT);
  double reflectance = (r0rth * r0rth + rPar * rPar) / 2;

  Point3D origin = ray.intersection.point;
  Vector3D direction = (n*cosI-cosT)*normal-(n*dir);
  direction.normalize();
  Ray3D refractionRay = Ray3D(origin, direction);
  refractionRay.refractionNumber = ray.refractionNumber + 1;
  refractionRay.startObject = ray.intersection.sceneObject;
  determineCurrentMedium(refractionRay, ray);

  return std::make_pair(refractionRay, reflectance);
}
