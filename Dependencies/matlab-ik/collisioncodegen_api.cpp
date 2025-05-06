/* Copyright 2019-2022 The MathWorks, Inc. */
#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include "collisioncodegen/collisioncodegen_api.hpp"
#include "collisioncodegen/collisioncodegen_CollisionGeometry.hpp"
#include "collisioncodegen/collisioncodegen_checkCollision_api.hpp"
#else
#include "collisioncodegen_CollisionGeometry.hpp"
#include "collisioncodegen_api.hpp"
#include "collisioncodegen_checkCollision_api.hpp"
#endif
#include <cstring>

CollisionGeometryVoidPtr collisioncodegen_makeBox(real64_T x, real64_T y,
                                                  real64_T z) {
  return static_cast<void *>(new shared_robotics::CollisionGeometry(
      static_cast<ccd_real_t>(x), static_cast<ccd_real_t>(y),
      static_cast<ccd_real_t>(z)));
}

CollisionGeometryVoidPtr collisioncodegen_makeSphere(real64_T r) {
  return static_cast<void *>(
      new shared_robotics::CollisionGeometry(static_cast<ccd_real_t>(r)));
}

CollisionGeometryVoidPtr collisioncodegen_makeCylinder(real64_T r, real64_T h) {
  return static_cast<void *>(new shared_robotics::CollisionGeometry(
      static_cast<ccd_real_t>(r), static_cast<ccd_real_t>(h)));
}

CollisionGeometryVoidPtr collisioncodegen_makeCapsule(real64_T r, real64_T h) {
  shared_robotics::CollisionGeometry *capsule =
      new shared_robotics::CollisionGeometry(static_cast<ccd_real_t>(r),
                                             static_cast<ccd_real_t>(h));
  capsule->setEnumType(shared_robotics::CollisionGeometry::Type::Capsule);
  return static_cast<void *>(capsule);
}

CollisionGeometryVoidPtr collisioncodegen_makeMesh(real64_T *vertices,
                                                   real64_T numVertices) {
  return static_cast<void *>(new shared_robotics::CollisionGeometry(
      static_cast<ccd_real_t *>(vertices), static_cast<int>(numVertices),
      true));
}

void collisioncodegen_destructGeometry(const CollisionGeometryVoidPtr *objPtr) {
  // The destructor accepts a pointer to a pointer. This is required because
  // of g2415822. We are destroying what objPtr holds, and not objPtr itself.
  // Here objPtr is read-only.
  if (*objPtr != nullptr) {
    delete static_cast<shared_robotics::CollisionGeometry *>(*objPtr);
  }
}

CollisionGeometryVoidPtr
collisioncodegen_copyGeometry(CollisionGeometryVoidPtr obj) {
  shared_robotics::CollisionGeometry *geom =
      static_cast<shared_robotics::CollisionGeometry *>(obj);
  return static_cast<void *>(new shared_robotics::CollisionGeometry(*geom));
}

int collisioncodegen_intersect(CollisionGeometryVoidPtr obj1, real64_T *pos1,
                               real64_T *quat1, CollisionGeometryVoidPtr obj2,
                               real64_T *pos2, real64_T *quat2,
                               real64_T computeDistance, real64_T *p1Vec,
                               real64_T *p2Vec, real64_T *distance) {
  shared_robotics::updatePose(obj1, static_cast<ccd_real_t *>(pos1),
                              static_cast<ccd_real_t *>(quat1));
  shared_robotics::updatePose(obj2, static_cast<ccd_real_t *>(pos2),
                              static_cast<ccd_real_t *>(quat2));
  return shared_robotics::intersect(
      obj1, obj2, static_cast<int>(computeDistance),
      static_cast<ccd_real_t *>(p1Vec), static_cast<ccd_real_t *>(p2Vec),
      *distance);
}

void collisioncodegen_getVertices(CollisionGeometryVoidPtr obj,
                                  real64_T *vertices) {
  auto geomvertices =
      static_cast<const shared_robotics::CollisionGeometry *>(obj)
          ->getVertices();
  auto numV = geomvertices.size();
  for (size_t i = 0; i < numV; ++i) {
    for (size_t j = 0; j < 3; j++) {
      vertices[numV * j + i] = geomvertices[i].v[j];
    }
  }
}

real64_T collisioncodegen_getNumVertices(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)
          ->getNumVertices());
}

real64_T collisioncodegen_getX(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)->getX());
}

real64_T collisioncodegen_getY(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)->getY());
}

real64_T collisioncodegen_getZ(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)->getZ());
}

real64_T collisioncodegen_getRadius(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)
          ->getRadius());
}

real64_T collisioncodegen_getLength(CollisionGeometryVoidPtr obj) {
  return static_cast<real64_T>(
      static_cast<const shared_robotics::CollisionGeometry *>(obj)
          ->getHeight());
}

real64_T collisioncodegen_getType(CollisionGeometryVoidPtr obj, char_T *type) {
  auto geomtype =
      static_cast<const shared_robotics::CollisionGeometry *>(obj)->getType();
  std::strcpy(static_cast<char *>(type), geomtype.c_str());
  return static_cast<real64_T>(geomtype.length());
}
