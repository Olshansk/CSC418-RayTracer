/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
        CSC418, SPRING 2005

    Implementations of functions in raytracer.h,
    and the main function which specifies the
    scene to be rendered.

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include "render_style.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sstream>

#define ADAPTIVE_SUBSAMPLING_THRESHOLD 0.01

Raytracer::Raytracer() : _lightSource(NULL) {
  _root = new SceneDagNode();
  antialias = false;
  antialias_rays = 0;
  depth_of_field = false;
  depth_of_field_rays = 0;
  depth_of_field_aperature = 0;
  depth_of_field_focus_plane = 1.0;
  max_reflection = 0;
  withShadows = false;
}

Raytracer::~Raytracer() {
  delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent,
    SceneObject* obj, Material* mat ) {
  SceneDagNode* node = new SceneDagNode( obj, mat );
  node->parent = parent;
  node->next = NULL;
  node->child = NULL;
  Colour randomSceneSignatureColour = Colour(RANDOM, RANDOM, RANDOM);
  node->scene_sig_col = randomSceneSignatureColour;
  // Add the object to the parent's child list, this means
  // whatever transformation applied to the parent will also
  // be applied to the child.
  if (parent->child == NULL) {
    parent->child = node;
  }
  else {
    parent = parent->child;
    while (parent->next != NULL) {
      parent = parent->next;
    }
    parent->next = node;
  }

  return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
  LightListNode* tmp = _lightSource;
  _lightSource = new LightListNode( light, tmp );
  return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
  Matrix4x4 rotation;
  double toRadian = 2*M_PI/360.0;
  int i;

  for (i = 0; i < 2; i++) {
    switch(axis) {
      case 'x':
        rotation[0][0] = 1;
        rotation[1][1] = cos(angle*toRadian);
        rotation[1][2] = -sin(angle*toRadian);
        rotation[2][1] = sin(angle*toRadian);
        rotation[2][2] = cos(angle*toRadian);
        rotation[3][3] = 1;
      break;
      case 'y':
        rotation[0][0] = cos(angle*toRadian);
        rotation[0][2] = sin(angle*toRadian);
        rotation[1][1] = 1;
        rotation[2][0] = -sin(angle*toRadian);
        rotation[2][2] = cos(angle*toRadian);
        rotation[3][3] = 1;
      break;
      case 'z':
        rotation[0][0] = cos(angle*toRadian);
        rotation[0][1] = -sin(angle*toRadian);
        rotation[1][0] = sin(angle*toRadian);
        rotation[1][1] = cos(angle*toRadian);
        rotation[2][2] = 1;
        rotation[3][3] = 1;
      break;
    }
    if (i == 0) {
        node->trans = node->trans*rotation;
      angle = -angle;
    }
    else {
      node->invtrans = rotation*node->invtrans;
    }
  }
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
  Matrix4x4 translation;

  translation[0][3] = trans[0];
  translation[1][3] = trans[1];
  translation[2][3] = trans[2];
  node->trans = node->trans*translation;
  translation[0][3] = -trans[0];
  translation[1][3] = -trans[1];
  translation[2][3] = -trans[2];
  node->invtrans = translation*node->invtrans;
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
  Matrix4x4 scale;

  scale[0][0] = factor[0];
  scale[0][3] = origin[0] - factor[0] * origin[0];
  scale[1][1] = factor[1];
  scale[1][3] = origin[1] - factor[1] * origin[1];
  scale[2][2] = factor[2];
  scale[2][3] = origin[2] - factor[2] * origin[2];
  node->trans = node->trans*scale;
  scale[0][0] = 1/factor[0];
  scale[0][3] = origin[0] - 1/factor[0] * origin[0];
  scale[1][1] = 1/factor[1];
  scale[1][3] = origin[1] - 1/factor[1] * origin[1];
  scale[2][2] = 1/factor[2];
  scale[2][3] = origin[2] - 1/factor[2] * origin[2];
  node->invtrans = scale*node->invtrans;
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view,
    Vector3D up ) {
  Matrix4x4 mat;
  Vector3D w;
  view.normalize();
  up = up - up.dot(view)*view;
  up.normalize();
  w = view.cross(up);

  mat[0][0] = w[0];
  mat[1][0] = w[1];
  mat[2][0] = w[2];
  mat[0][1] = up[0];
  mat[1][1] = up[1];
  mat[2][1] = up[2];
  mat[0][2] = -view[0];
  mat[1][2] = -view[1];
  mat[2][2] = -view[2];
  mat[0][3] = eye[0];
  mat[1][3] = eye[1];
  mat[2][3] = eye[2];

  return mat;
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray, Matrix4x4 modelToWorld, Matrix4x4 worldToModel) {
  SceneDagNode *childPtr;
  // Applies transformation of the current node to the global
  // transformation matrices.
  modelToWorld = modelToWorld*node->trans;
  worldToModel = node->invtrans*worldToModel;
  if (node->obj) {
    // Perform intersection.
    if (node->obj->intersect(ray, worldToModel, modelToWorld)) {
      ray.intersection.mat = node->mat;
      if (RenderStyle::rstyle == SCENE_SIGNATURE) {
        ray.col = node->scene_sig_col;
      }
    }
  }
  // Traverse the children.
  childPtr = node->child;
  while (childPtr != NULL) {
    traverseScene(childPtr, ray, modelToWorld, worldToModel);
    childPtr = childPtr->next;
  }

  // Removes transformation of the current node from the global
  // transformation matrices.
  worldToModel = node->trans*worldToModel;
  modelToWorld = modelToWorld*node->invtrans;
}

Colour Raytracer::reflectionColor( Ray3D& ray ) {
  Ray3D reflectionRay = LightSource::getReflectionRay(ray);
  return shadeRay(reflectionRay);
}

bool Raytracer::isIntersectionInShadow( Ray3D& ray, LightSource* light ) {
    Ray3D shadowRay = light->getShadowRay(ray);
    traverseScene(_root, shadowRay, Matrix4x4(), Matrix4x4());
    return (!shadowRay.intersection.none && shadowRay.intersection.t_value > 0 && shadowRay.intersection.t_value <= 1);
}

// Potentially TODO: Normalize t_value somehow.
void Raytracer::applyReflection( Ray3D& ray ) {
    if (ray.reflectionNumber < max_reflection) {
      Colour reflectionColour = reflectionColor(ray);
      ray.col = ray.col + ray.intersection.mat->reflection * exp (- ray.intersection.mat->ref_damping * ray.intersection.t_value) * reflectionColour;
      ray.col.clamp();
    }
}

void Raytracer::computeShading( Ray3D& ray ) {
  Colour ambientCol = Colour();
  int num_lights = 0;
  for (LightListNode* curLight = _lightSource; curLight != NULL; curLight = curLight->next) {
    if (curLight->light->hasAmbient()) {
      ambientCol = ambientCol + curLight->light->shadeAmbient(ray.intersection.mat);
      num_lights ++;
    }
  }

  if (num_lights != 0) {
    ray.col = ray.col + ambientCol / double(num_lights);
  }

  for (LightListNode* curLight = _lightSource; curLight != NULL; curLight = curLight->next) {
    // Each lightSource provides its own shading function.
    curLight->light->shade(ray, withShadows && isIntersectionInShadow(ray, curLight->light));
  }

  // Appply secondary reflection
  applyReflection(ray);
}

void Raytracer::initPixelBuffer() {
  int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
  _rbuffer = new unsigned char[numbytes];
  _gbuffer = new unsigned char[numbytes];
  _bbuffer = new unsigned char[numbytes];
  for (int i = 0; i < _scrHeight; i++) {
    for (int j = 0; j < _scrWidth; j++) {
      _rbuffer[i*_scrWidth+j] = 0;
      _gbuffer[i*_scrWidth+j] = 0;
      _bbuffer[i*_scrWidth+j] = 0;
    }
  }
}

void Raytracer::flushPixelBuffer( char *file_name ) {
  bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer);
  delete _rbuffer;
  delete _gbuffer;
  delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray ) {
  Colour col(0.0, 0.0, 0.0);
  traverseScene(_root, ray, Matrix4x4(), Matrix4x4());
  // Don't bother shading if the ray didn't hit
  // anything.
  if (!ray.intersection.none) {
    if (RenderStyle::rstyle == PHONG || RenderStyle::rstyle == AMBIENT_DIFFUSE) {
      computeShading(ray);
    }
    col = ray.col;
  }
  // You'll want to call shadeRay recursively (with a different ray,
  // of course) here to implement reflection/refraction effects.

  return col;
}

Colour Raytracer::shadeSingleViewRay(Matrix4x4 viewToWorld, Point3D imagePlane, Point3D origin) {
  Vector3D direction = imagePlane - origin;
  direction = viewToWorld * direction;
  direction.normalize();
  origin = viewToWorld * origin;
  Ray3D ray = Ray3D(origin, direction);
  return shadeRay(ray);
}

Colour Raytracer::shadeViewRay(Matrix4x4 viewToWorld, Point3D imagePlane, Point3D origin) {
  if (depth_of_field) {
    Colour col = Colour();
    for (int m = 0; m < depth_of_field_rays; m++) {
      Point3D origin = Point3D((RANDOM - 0.5) * depth_of_field_aperature, (RANDOM - 0.5) * depth_of_field_aperature, 0);
      col = col + shadeSingleViewRay(viewToWorld, imagePlane, origin);
    }
    return col / depth_of_field_rays;
  } else {
    return shadeSingleViewRay(viewToWorld, imagePlane, origin);
  }
}

Colour Raytracer::subsampleRay(Point3D imagePlaneOrig, Point3D imagePlane, double factor,
      Matrix4x4 viewToWorld, Point3D origin) {
  Colour col = Colour();
  for (int k = 0; k < antialias_rays; k++) {
    imagePlane[0] = imagePlaneOrig[0] + RANDOM / factor;
    imagePlane[1] = imagePlaneOrig[1] + RANDOM / factor;

    col = col + shadeViewRay(viewToWorld, imagePlane, origin);
  }

  return col;
}

void Raytracer::render( int width, int height, Point3D eye, Vector3D view,
    Vector3D up, double fov, int scene_num ) {
  Matrix4x4 viewToWorld;
  _scrWidth = width;
  _scrHeight = height;
  double factor = (double(height / depth_of_field_focus_plane)/2)/tan(fov*M_PI/360.0);

  initPixelBuffer();
  viewToWorld = initInvViewMatrix(eye, view, up);

  int rows_completed = 0;

  // Construct a ray for each pixel.
  #pragma omp parallel for
  for (int i = 0; i < _scrHeight; i++) {
    Colour (*aaCache)[2];
    if (antialias && antialias_rays > 4) {
      aaCache = new Colour[_scrWidth][2];
    }

    for (int j = 0; j < _scrWidth; j++) {
      // Sets up ray origin and direction in view space,
      // image plane is at z = -1.

      Point3D origin(0, 0, 0);
      Point3D imagePlaneOrig, imagePlane;
      imagePlaneOrig[0] = (-double(width)/2 + j)/factor;
      imagePlaneOrig[1] = (-double(height)/2 + i)/factor;
      imagePlaneOrig[2] = -depth_of_field_focus_plane;

      imagePlane[2] = imagePlaneOrig[2];

      Colour col;
      if (antialias) {
        if (antialias_rays > 4) {
          // Bottom left pixel
          Colour col1;
          if (j != 0) {
            col1 = aaCache[j - 1][0];
          } else {
            imagePlane[0] = imagePlaneOrig[0];
            imagePlane[1] = imagePlaneOrig[1];
            col1 = shadeViewRay(viewToWorld, imagePlane, origin);
          }

          // Bottom right pixel
          imagePlane[0] = imagePlaneOrig[0] + 1.0 / factor;
          imagePlane[1] = imagePlaneOrig[1];
          Colour col2 = shadeViewRay(viewToWorld, imagePlane, origin);
          aaCache[j][0] = col2;

          // Top right pixel
          imagePlane[0] = imagePlaneOrig[0] + 1.0 / factor;
          imagePlane[1] = imagePlaneOrig[1] + 1.0 / factor;
          Colour col3 = shadeViewRay(viewToWorld, imagePlane, origin);
          aaCache[j][1] = col3;

          // Top left pixel
          Colour col4;
          if (j != 0) {
            col4 = aaCache[j - 1][1];
          } else {
            imagePlane[0] = imagePlaneOrig[0];
            imagePlane[1] = imagePlaneOrig[1] + 1.0 / factor;
            col4 = shadeViewRay(viewToWorld, imagePlane, origin);
          }

          // Check if any of the corner pixels are substantially different than each other
          float threshold = ADAPTIVE_SUBSAMPLING_THRESHOLD;
          bool cornersDifferent =
              colourDiff(col1, col2, threshold) || colourDiff(col1, col3, threshold) ||
              colourDiff(col1, col4, threshold) || colourDiff(col2, col3, threshold) ||
              colourDiff(col2, col4, threshold) || colourDiff(col3, col4, threshold);

          if (cornersDifferent) {
            col = subsampleRay(imagePlaneOrig, imagePlane, factor, viewToWorld, origin);
            col = col + col1 + col2 + col3 + col4;
            col = col / (antialias_rays + 4.0);
          } else {
            col = (col1 + col2 + col3 + col4) / 4.0;
          }
        } else {
          col = subsampleRay(imagePlaneOrig, imagePlane, factor, viewToWorld, origin);
          col = col / antialias_rays;
        }
      } else {
        imagePlane[0] = imagePlaneOrig[0] + 0.5 / factor;
        imagePlane[1] = imagePlaneOrig[1] + 0.5 / factor;
        col = shadeViewRay(viewToWorld, imagePlane, origin);
      }

      int index = i*width+j;

      _rbuffer[index] = int(col[0]*255);
      _gbuffer[index] = int(col[1]*255);
      _bbuffer[index] = int(col[2]*255);
    }

    #pragma omp critical
    {
      rows_completed++;
      if (rows_completed % 10 == 0) {
        printf("Columns completed: %d\n", rows_completed);
      }
    }
  }

  std::stringstream sstm;
  sstm << "scene" << scene_num << ".bmp";
  flushPixelBuffer(strdup(sstm.str().c_str()));
}

void printUsage() {
  printf(
    "Usage: raytracer [options]\n"
    "\n"
    "Options:\n"
    "--help                        print this message\n"
    "--scene-signature             render a scene signature\n"
    "--ambient-diffuse             render a scene with only the diffuse and ambient\n"
    "                              components of the Phong model\n"
    "--phong                       render a scene with all three terms of the Phong model\n"
    "    You must use one of the 3 rendering modes.\n"
    "\n"
    "--scene 1                     pick which scene to render"
    "--width 320                   width of image to render\n"
    "--height 240                  height of image to render\n"
    "--antialias 4                 number of rays to use for antialias subsampling\n"
    "--depth-of-field 60 0.05 1    number of rays, aperature size (bigger the blurier), focal plane distance\n"
    "--reflection 1                number of reflection rays for each collision opint (bigger number creates\n"
    "                              a more accurate reflection). The default values is 0.\n"
    "--shadows                     including this argument adds shadows\n"
  );
}

int contains_option(int argc, char* argv[], const char* option) {
  for (int x = 0; x < argc; x++) {
    if (strcmp(argv[x], option) == 0) {
      return x;
    }
  }

  return -1;
}

int main(int argc, char* argv[])
{
  // Build your scene and setup your camera here, by calling
  // functions from Raytracer.  The code here sets up an example
  // scene and renders it from two different view points, DO NOT
  // change this if you're just implementing part one of the
  // assignment.
  Raytracer raytracer;
  int width = 320;
  int height = 240;

  int width_arg = contains_option(argc, argv, "--width");
  if (width_arg > 0) {
    width = atoi(argv[width_arg + 1]);
    printf("Rendering image of width %dpx.\n", width);
  }

  int height_arg = contains_option(argc, argv, "--height");
  if (height_arg > 0) {
    height = atoi(argv[height_arg + 1]);
    printf("Rendering image of height %dpx.\n", height);
  }

  if (contains_option(argc, argv, "--help") > 0) {
    printUsage();
    return 0;
  }

  // Handle command line arguments
  if (argc > 1) {
    if (contains_option(argc, argv, "--scene-signature") > 0) {
      RenderStyle::rstyle = SCENE_SIGNATURE;
      printf("Rendering only scene signature.\n");
    } else if (contains_option(argc, argv, "--ambient-diffuse") > 0) {
      RenderStyle::rstyle = AMBIENT_DIFFUSE;
      printf("Rendering ambiant and diffuse light modelled image.\n");
    } else if (contains_option(argc, argv, "--phong") > 0) {
      RenderStyle::rstyle = PHONG;
      printf("Rendering phong light modelled image.\n");
    } else {
      printUsage();
      return 0;
    }
  } else {
    printUsage();
    return 0;
  }

  int antialias_arg = contains_option(argc, argv, "--antialias");
  if (antialias_arg > 0) {
    raytracer.antialias = true;
    raytracer.antialias_rays = atoi(argv[antialias_arg + 1]);
    printf("Using %d antialiasing rays.\n", raytracer.antialias_rays);
  }

  int depth_of_field_arg = contains_option(argc, argv, "--depth-of-field");
  if (depth_of_field_arg > 0) {
    raytracer.depth_of_field = true;
    raytracer.depth_of_field_rays = atoi(argv[depth_of_field_arg + 1]);
    raytracer.depth_of_field_aperature = atof(argv[depth_of_field_arg + 2]);
    raytracer.depth_of_field_focus_plane = atof(argv[depth_of_field_arg + 3]);
    printf("Using %d depth of field rays.\n", raytracer.depth_of_field_rays);
    printf("Using %f aperature size.\n", raytracer.depth_of_field_aperature);
    printf("Using focus plane %f units away from origin.\n", raytracer.depth_of_field_focus_plane);
  }

  int reflection_arg = contains_option(argc, argv, "--reflection");
  if (reflection_arg > 0) {
    raytracer.max_reflection = atoi(argv[reflection_arg + 1]);
    printf("Maximum reflection number is %d.\n", raytracer.max_reflection);
  }

  int scene_num_arg = contains_option(argc, argv, "--scene");
  int scene_num;
  if (scene_num_arg > 0) {
    scene_num = atoi(argv[scene_num_arg + 1]);
    printf("Rendering scene #%d.\n", scene_num);
  } else {
    printUsage();
    return 0;
  }

  int shadows_arg = contains_option(argc, argv, "--shadows");
  if (shadows_arg > 0) {
    raytracer.withShadows = true;
    printf("Rendering scene with shadows.\n");
  }


  printf("\n");

  // Defines a material for shading.
  Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648),
      Colour(0.628281, 0.555802, 0.366065),
      51.2, 0.3, 0.2);
  Material ruby( Colour(0.1745, 0.01175, 0.01175), Colour(0.61424, 0.04136, 0.04136),
      Colour(0.727811, 0.626959, 0.626959),
      51.2, 0, 0 );
  Material emerald( Colour(0.0215, 0.1745, 0.0215), Colour(0.07568, 0.61424, 0.07568),
      Colour(0.633, 0.727811, 0.633),
      51.2, 0, 0 );
  Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63),
      Colour(0.316228, 0.316228, 0.316228),
      12.8, 0.3, 0.2 );

  if (scene_num == 1) {
    // Camera parameters.
    Point3D eye(0, 0, 1);
    Vector3D view(0, 0, -1);
    Vector3D up(0, 1, 0);
    double fov = 60;

    // Defines a point light source.
    raytracer.addLightSource( new PointLight(Point3D(0, 0, 5),
          Colour(0.9, 0.9, 0.9) ) );

    // Add a unit square into the scene with material mat.
    SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
    SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );

    // Apply some transformations to the unit square.
    double factor1[3] = { 1.0, 2.0, 1.0 };
    double factor2[3] = { 6.0, 6.0, 6.0 };
    raytracer.translate(sphere, Vector3D(0, 0, -5));
    raytracer.rotate(sphere, 'x', -45);
    raytracer.rotate(sphere, 'z', 45);
    raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

    raytracer.translate(plane, Vector3D(0, 0, -7));
    raytracer.rotate(plane, 'z', 45);
    raytracer.scale(plane, Point3D(0, 0, 0), factor2);

    // Render the scene
    raytracer.render(width, height, eye, view, up, fov, scene_num);
  } else if (scene_num == 2) {
    // Render it from a different point of view.
    Point3D eye(4, 2, 1);
    Vector3D view(-4, -2, -6);
    Vector3D up(0, 1, 0);
    double fov = 60;

    // Defines a point light source.
    raytracer.addLightSource( new PointLight(Point3D(0, 0, 5),
          Colour(0.9, 0.9, 0.9) ) );

    // Add a unit square into the scene with material mat.
    SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
    SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );

    // Apply some transformations to the unit square.
    double factor1[3] = { 1.0, 2.0, 1.0 };
    double factor2[3] = { 6.0, 6.0, 6.0 };
    raytracer.translate(sphere, Vector3D(0, 0, -5));
    raytracer.rotate(sphere, 'x', -45);
    raytracer.rotate(sphere, 'z', 45);
    raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

    raytracer.translate(plane, Vector3D(0, 0, -7));
    raytracer.rotate(plane, 'z', 45);
    raytracer.scale(plane, Point3D(0, 0, 0), factor2);
    raytracer.render(width, height, eye, view, up, fov, scene_num);
  } else if (scene_num == 3) {
    Point3D eye(0, 1, 1);
    Vector3D view(0, 0, -1);
    Vector3D up(0, 1, 0);
    double fov = 60;

    raytracer.addLightSource( new PointLight(Point3D(-5, 5, 5),
          Colour(0.9, 0.9, 0.9) ) );

    SceneDagNode* sphere1 = raytracer.addObject( new UnitSphere(), &ruby );
    SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &emerald );
    SceneDagNode* sphere3 = raytracer.addObject( new UnitSphere(), &gold );

    raytracer.translate(sphere1, Vector3D(0, 0, -5));
    raytracer.translate(sphere2, Vector3D(3.2, 0, -11));
    raytracer.translate(sphere3, Vector3D(-3.2, 0, -8));

    // Render the scene
    raytracer.render(width, height, eye, view, up, fov, scene_num);
  } else if (scene_num == 4) {
    // Defines a point light source.
    Point3D eye(0, 0, 1);
    Vector3D view(0, 0, -1);
    Vector3D up(0, 1, 0);
    double fov = 60;

    raytracer.addLightSource( new PointLight(Point3D(0, 0, 5),
          Colour(0.9, 0.9, 0.9) ) );

    // Add a unit square into the scene with material mat.
    // SceneDagNode* sphere = raytracer.addObject( new GeneralQuadratic(1, 1, 1, 0, 0, 0, 0, 0, 0, -1), &gold );
    SceneDagNode* cone = raytracer.addObject( new GeneralQuadratic(1, -1, 1, 0, 0, 0, 0, 0, 0, -1), &gold );
    // SceneDagNode* cylinder = raytracer.addObject( new GeneralQuadratic(1, 0, 1, 0, 0, 0, 0, 0, 0, -1), &gold );
    SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );

    // Apply some transformations to the unit square.

    double factor1[3] = { 1.0, 3.0, 1.0 };
    double factor2[3] = { 6.0, 6.0, 6.0 };
    double factor3[3] = { 1, 1, 1 };

    // raytracer.translate(sphere, Vector3D(0, 0, -5));
    // raytracer.rotate(sphere, 'x', -45);
    // raytracer.rotate(sphere, 'z', 45);
    // raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

    raytracer.translate(cone, Vector3D(0, 0, -5));
    // raytracer.rotate(cone, 'x', -45);
    // raytracer.rotate(cone, 'z', 45);
    raytracer.scale(cone, Point3D(0, 0, 0), factor1);

    // raytracer.translate(cylinder, Vector3D(0, 0, -5));
    // raytracer.rotate(cylinder, 'x', -45);
    // raytracer.rotate(cylinder, 'z', 45);
    // raytracer.scale(cylinder, Point3D(0, 0, 0), factor3);

    raytracer.translate(plane, Vector3D(0, 0, -7));
    raytracer.rotate(plane, 'z', 45);
    raytracer.scale(plane, Point3D(0, 0, 0), factor2);

    // Render the scene, feel free to make the image smaller for
    // testing purposes.
    raytracer.render(width, height, eye, view, up, fov, scene_num);

    // Render it from a different point of view.
    Point3D eye2(4, 2, 1);
    Vector3D view2(-4, -2, -6);
    raytracer.render(width, height, eye2, view2, up, fov, scene_num);
  }

  return 0;
}
