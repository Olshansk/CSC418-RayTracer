/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

       light source classes

***********************************************************/

#include <utility>
#include "util.h"

// Base class for a light source.  You could define different types
// of lights here, but point light is sufficient for most scenes you
// might want to render.  Different light sources shade the ray
// differently.
class LightSource {
public:
  static std::pair <Ray3D,double> getRefractionRay( Ray3D& ray );
  static Ray3D getReflectionRay( Ray3D& ray );
  virtual Ray3D getShadowRay( Ray3D& ray ) = 0;
  virtual void shade( Ray3D&, int glossy_rays ) = 0;
  virtual Point3D get_position() const = 0;
  // Determine if a light contributes to the ambient light of the entire scene
  virtual bool hasAmbient() { return false; }
  // Get the colour that the light contributes to the ambient light of the entire scene
  virtual Colour shadeAmbient(Material* mat) {}
};

// A point light is defined by its position in world space and its
// colour.
class PointLight : public LightSource {
public:
  PointLight( Point3D pos, Colour col ) : _pos(pos), _col_ambient(col),
  _col_diffuse(col), _col_specular(col) {}
  PointLight( Point3D pos, Colour ambient, Colour diffuse, Colour specular )
  : _pos(pos), _col_ambient(ambient), _col_diffuse(diffuse),
  _col_specular(specular) {}
  void shade( Ray3D& ray, int glossy_rays);
  Ray3D getShadowRay( Ray3D& ray );
  Point3D get_position() const { return _pos; }
  bool hasAmbient() { return true; }
  Colour shadeAmbient(Material* mat);
  // Get specular colour given the indicent vector and the perfect reflection vector (r_vec)
  Colour shadeSpecular(Vector3D r_vec, Vector3D incident_vec, Material* mat) ;

private:
  Point3D _pos;
  Colour _col_ambient;
  Colour _col_diffuse;
  Colour _col_specular;
};
