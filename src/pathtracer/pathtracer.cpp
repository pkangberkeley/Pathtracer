#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading

  for (int i = 0; i < num_samples; i++) {
      Vector3D w_in = hemisphereSampler -> get_sample();
      Vector3D w_in_world = o2w * w_in;
      Ray raySample = Ray(hit_p, w_in_world);
      raySample.min_t = EPS_F;
      Intersection new_isect;
      if (bvh -> intersect(raySample, &new_isect)) {
          Spectrum emission = new_isect.bsdf -> get_emission();
          L_out += emission * isect.bsdf -> f(w_out, w_in) * cos_theta(w_in) * 2.0 * PI;
      }
  }
  return L_out / num_samples;
}

Spectrum PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  // New Code
  int num_samples;
  float distToLight, pdf;
  Vector3D wi;
  // Iterate through each light
  for (SceneLight *light : scene -> lights) {
      Spectrum L_light;
      if (light -> is_delta_light()) {
          num_samples = 1;
      } else {
          num_samples = ns_area_light;
      }
      // Sample
      for (int i = 0; i < num_samples; i++) {
          Spectrum lightSample = light -> sample_L(hit_p, &wi, &distToLight, &pdf);
          Vector3D w_in = w2o * wi;
          if (w_in.z >= 0) {
              Vector3D w_in_world = o2w * w_in;
              Ray raySample = Ray(hit_p, wi);
              raySample.max_t = distToLight - EPS_F;
              raySample.min_t = EPS_F;
              Intersection new_isect;
              if (!bvh -> intersect(raySample, &new_isect)) {
                  L_light += (lightSample * cos_theta(w_in) * isect.bsdf -> f(w_out, w_in) * w_in.z) / pdf;
              }
          }
      }
      L_out += L_light / (double) num_samples;
  }
  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf -> get_emission();
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
      return estimate_direct_lighting_hemisphere(r, isect);
  }
  return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Spectrum L_out(0, 0, 0);

  L_out += one_bounce_radiance(r, isect);
  float pdf;
  float cont_prob = 0.7;
  Vector3D w_in;
  Spectrum sampleF = isect.bsdf->sample_f(w_out, &w_in, &pdf);

  if(max_ray_depth <= 1){
      return L_out;
  } else if (max_ray_depth > 1 && r.depth == max_ray_depth) {
      Vector3D w_in_world = o2w * w_in;
      Ray new_ray = Ray(hit_p, w_in_world);
      new_ray.depth = r.depth - 1;
      new_ray.min_t = EPS_F;
      Intersection new_isect;
      if (bvh->intersect(new_ray, &new_isect)) {
          Spectrum new_rad = at_least_one_bounce_radiance(new_ray, new_isect);
          L_out += new_rad * sampleF * cos_theta(w_in) / (pdf);
      }
  } else if (coin_flip(cont_prob) && r.depth > 1) {
      Vector3D w_in_world = o2w * w_in;
      Ray new_ray = Ray(hit_p, w_in_world);
      new_ray.depth = r.depth - 1;
      new_ray.min_t = EPS_F;
      Intersection new_isect;
      if (bvh->intersect(new_ray, &new_isect)){
          Spectrum new_rad = at_least_one_bounce_radiance(new_ray, new_isect);
          L_out += new_rad * sampleF * cos_theta(w_in) / (pdf * cont_prob);
      }
  }
  return L_out;
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  if (!bvh->intersect(r, &isect)) {
      return L_out;
  }
  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.


  if (PART == 2) {
      L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
      return L_out;
  }

  // TODO (Part 3): Return the direct illumination.
  // Direct lighting importance test
  if (PART == 3) {
      L_out = zero_bounce_radiance(r, isect);
      L_out += one_bounce_radiance(r, isect);
      return L_out;
  }
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  L_out = zero_bounce_radiance(r, isect);
  if (max_ray_depth > 0) {
      L_out += at_least_one_bounce_radiance(r, isect);
  }
  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Spectrum avg_spectrum = Spectrum();
  int n = 0;
  double s1 = 0;
  double s2 = 0;
  bool c = false;
  int index = 0;
  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  while (!c && index < num_samples / samplesPerBatch) {
      for (int i = 0; i < samplesPerBatch; i++) {
          Vector2D grid_sample = origin + gridSampler->get_sample();
          double normalized_x = (grid_sample.x) / sampleBuffer.w;
          double normalized_y = (grid_sample.y) / sampleBuffer.h;
          Ray ray = camera->generate_ray(normalized_x, normalized_y);
          ray.depth = max_ray_depth;
          Spectrum ray_radiance = est_radiance_global_illumination(ray);
          avg_spectrum += ray_radiance;
          s1 += ray_radiance.illum();
          s2 += ray_radiance.illum() * ray_radiance.illum();
      }
      n += (int) samplesPerBatch;
      double mu = s1 / n;
      double sigma_sq = ((double) 1.0 / (n - 1)) * (s2 - (s1 * s1) / n);
      double I = 1.96 * sqrt(sigma_sq / n);
      if (I <= maxTolerance * mu) {
          c = true;
          sampleCountBuffer[x + y * sampleBuffer.w] = n;
      }
      index++;
  }
  if (!c) {
      for (int i = 0; i < num_samples % samplesPerBatch; i++) {
          Vector2D grid_sample = origin + gridSampler->get_sample();
          double normalized_x = (grid_sample.x) / sampleBuffer.w;
          double normalized_y = (grid_sample.y) / sampleBuffer.h;
          Ray ray = camera->generate_ray(normalized_x, normalized_y);
          ray.depth = max_ray_depth;
          Spectrum ray_radiance = est_radiance_global_illumination(ray);
          avg_spectrum += ray_radiance;
      }
      n += num_samples % samplesPerBatch;
  }
  sampleBuffer.update_pixel(avg_spectrum / n, x, y);
}

} // namespace CGL