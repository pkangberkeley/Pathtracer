#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  BBox bbox;
  int count = 0;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    count++;
  }

  BVHNode *node = new BVHNode(bbox);
  if (count <= max_leaf_size) {
      node -> start = start;
      node -> end = end;
      return node;
  }

  // Calculate the largest axis to split
  double axis = (bbox.extent.x < bbox.extent.y) ? bbox.extent.y : bbox.extent.x;
  axis = (axis < bbox.extent.z) ? bbox.extent.z : axis;

  // Calculate the average centroid
  double avg = 0;
  for (auto p = start; p != end; p++) {
      avg += (*p)->get_bbox().centroid()[axis];
  }
  avg = avg / count;

  // Divide primitives into left and right collection based on centroid avg
  auto *left = new vector<Primitive *>();
  auto *right = new vector<Primitive *>();
  for (auto p = start; p != end; p++) {
    if ((*p)->get_bbox().centroid()[axis] < avg) {
        left -> push_back(*p);
    } else {
        right -> push_back(*p);
    }
  }

  // Check if one of the collections is empty and return it as a leaf
  if (left -> empty() || right -> empty()) {
      node -> start = start;
      node -> end = end;
      return node;
  }

  // recursively call construct_bvh on left and right collections
  node -> l = construct_bvh(left -> begin(), left -> end(), max_leaf_size);
  node -> r = construct_bvh(right -> begin(), right -> end(), max_leaf_size);
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  if (PART == 2.5) {
      // Brute Force Algorithm
      for (auto p : primitives) {
          total_isects++;
          if (p -> has_intersection(ray))
              return true;
      }
      return false;
  }
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node -> bb.intersect(ray, t0, t1)) {
      return false;
  }
  if (t1 < ray.min_t || t0 > ray.max_t){
      return false;
  }
  if (node -> isLeaf()) {
      for (auto p = node -> start; p != node -> end; p++) {
          total_isects++;
          if ((*p) -> has_intersection(ray))
              return true;
      }
      return false;
  }
  return (has_intersection(ray, node -> l) || has_intersection(ray, node -> r));
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Brute Force Algorithm
  if (PART == 2.5) {
      bool hit = false;
      for (auto p: primitives) {
          total_isects++;
          hit = p -> intersect(ray, i) || hit;
      }
      return hit;
  }

  // Fill in the intersect function.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node -> bb.intersect(ray, t0, t1)) {
      return false;
  }
  if (t0 > ray.max_t || t1 < ray.min_t) {
      return false;
  }

  bool hit = false;
  if (node -> isLeaf()) {
      for (auto p = node -> start; p != node -> end; p++) {
          total_isects++;
          hit = (*p) -> intersect(ray, i) || hit;
      }
      return hit;
  }
  bool hit1 = intersect(ray, i, node -> l);
  bool hit2 = intersect(ray, i, node -> r);
  return (hit1 || hit2);
}
} // namespace SceneObjects
} // namespace CGL
