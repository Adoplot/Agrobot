// Copyright 2018-2019 The MathWorks, Inc.

#include <vector>
#include <iostream>

#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include "collisioncodegen/collisioncodegen_checkCollision_api.hpp"
#include <ccd/ccd_ccd.h>
#else // during portable codegen, all files are placed in a flat directory
#include "collisioncodegen_checkCollision_api.hpp"
#include <ccd_ccd.h>
#endif


using namespace shared_robotics;

static void fSupport(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* supportPoint);

EXTERN_C COLLISIONCODEGEN_API int shared_robotics::intersect(const void* obj1, const void* obj2, int computeDistance, ccd_real_t* p1Vec, ccd_real_t* p2Vec, ccd_real_t& distance)
{
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct
        ccd_support_fn f = &fSupport;
        ccd.support1 = f;
        ccd.support2 = f;
        ccd.max_iterations = 100; // maximal number of iterations

        int result;
        ccd_vec3_t p1 = {{0, 0, 0}};
        ccd_vec3_t p2 = {{0, 0, 0}};
        distance = -CCD_ONE;
        p1Vec[0] = 0;
        p1Vec[1] = 0;
        p1Vec[2] = 0;

        p2Vec[0] = 0;
        p2Vec[1] = 0;
        p2Vec[2] = 0;

        // std::cout << "computeDistance = " <<computeDistance << std::endl;
        if (computeDistance) 
        {
            // returns -1 if two bodies intersect, otherwise returns the minimal distance
            distance = ccdDistance(obj1, obj2, &ccd, &p1, &p2);

            p1Vec[0] = p1.v[0];
            p1Vec[1] = p1.v[1];
            p1Vec[2] = p1.v[2];

            p2Vec[0] = p2.v[0];
            p2Vec[1] = p2.v[1];
            p2Vec[2] = p2.v[2];

            if (distance < 0) 
            {
                result = 1;
            } 
            else 
            {
                result = 0;
            }
        } 
        else 
        {
            // returns 1 if two objects intersect
            result = ccdGJKIntersect(obj1, obj2, &ccd);
        }

        return result;
}

EXTERN_C COLLISIONCODEGEN_API void shared_robotics::updatePose(void* obj, const ccd_real_t* position, const ccd_real_t* orientation) 
{
    auto geom = static_cast<CollisionGeometry*>(obj);
    ccdVec3Set(&geom->m_pos, static_cast<ccd_real_t>(position[0]), static_cast<ccd_real_t>(position[1]), static_cast<ccd_real_t>(position[2]));
    ccdQuatSet(&geom->m_quat, static_cast<ccd_real_t>(orientation[1]), static_cast<ccd_real_t>(orientation[2]), static_cast<ccd_real_t>(orientation[3]), static_cast<ccd_real_t>(orientation[0])); // x,y,z,w in ccd
}

static void fSupport(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* supportPoint)
{
    auto geomPtr = (static_cast<const CollisionGeometry*>(obj));
    geomPtr->support(dir, supportPoint);
}
