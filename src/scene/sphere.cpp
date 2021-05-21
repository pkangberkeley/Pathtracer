#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - this -> o, r.d);
  double c = dot(r.o - this -> o, r.o - this -> o) - this -> r2;
  double discriminant = pow(b, 2) - 4 * a * c;
  if (discriminant >= 0) {
      double root1 = (-b + sqrt(discriminant)) / (2 * a);
      double root2 = (-b - sqrt(discriminant)) / (2 * a);
      t1 = min(root1, root2);
      t2 = max(root1, root2);
      return true;
  }
  return false;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1;
  double t2;
  if (test(r, t1, t2) && r.min_t <= t1 && r.max_t >= t1) {
      r.max_t = t1;
      return true;
  } else if (test(r, t1, t2) && r.min_t <= t2 && r.max_t >= t2) {
      r.max_t = t2;
      return true;
  }
  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  if (has_intersection(r)) {
      i -> t = r.max_t;
      Vector3D normal = ((r.o + i -> t * r.d) - (this -> o));
      normal.normalize();
      i -> n = normal;
      i -> primitive = this;
      i -> bsdf = get_bsdf();
      return true;
  }
  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
