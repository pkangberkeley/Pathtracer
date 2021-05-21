#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  double t_min_x = (min.x - r.o.x) / r.d.x;
  double t_max_x = (max.x - r.o.x) / r.d.x;
  double t_min_y = (min.y - r.o.y) / r.d.y;
  double t_max_y = (max.y - r.o.y) / r.d.y;
  double t_min_z = (min.z - r.o.z) / r.d.z;
  double t_max_z = (max.z - r.o.z) / r.d.z;

  // Account for cases where max < min
  double temp;
  if (t_max_x < t_min_x) {
      std::swap(t_min_x, t_max_x);
  }
  if (t_max_y < t_min_y) {
      std::swap(t_min_y, t_max_y);
  }
  if (t_max_z < t_min_z) {
      std::swap(t_min_z, t_max_z);
  }
  // find the max of the mins
  t0 = (t_min_x < t_min_y) ? t_min_y : t_min_x;
  t0 = (t0 < t_min_z) ? t_min_z : t0;
  // find the min of the maxs
  t1 = (t_max_x < t_max_y) ? t_max_x : t_max_y;
  t1 = (t1 < t_max_z) ? t1 : t_max_z;
  return t0 <= t1 && t1 >= 0;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
