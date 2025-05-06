// Copyright 2018-2022 The MathWorks, Inc.

#ifdef BUILDING_LIBMWCOLLISIONCODEGEN
#include "collisioncodegen/collisioncodegen_CollisionGeometry.hpp"
#else // during portable codegen, all files are placed in a flat directory
#include "collisioncodegen_CollisionGeometry.hpp"
#endif

#include <iostream>
#include <algorithm>

namespace shared_robotics {
void CollisionGeometryDeleter::operator()(CollisionGeometry* ptr) {
    delete ptr;
}
} // namespace shared_robotics

namespace shared_robotics {
CollisionGeometry::~CollisionGeometry() {
    // std::cout << "A collision geometry of type " << getType() << " is destroyed" << std::endl;
}


/// compute the support point with interface to libccd
void CollisionGeometry::support(const ccd_vec3_t* ccdDir, ccd_vec3_t* ccdSupportVertex) const {
    // dir should always have non-zero length

    ccd_quat_t quatInv = {{0, 0, 0, 1}};
    ccdQuatInvert2(&quatInv, &m_quat);
    ccd_vec3_t dirRot = {{0, 0, 0}};
    ccd_vec3_t vert = {{0, 0, 0}};

    ccdVec3Copy(&dirRot, ccdDir);
    ccdQuatRotVec(&dirRot, &quatInv); // rotate dir into geometry's coordinates

    switch (m_type) {
    case Type::Box: // for box primitive
    {
        ccdVec3Set(&vert, CollisionGeometry::sign(dirRot.v[0]) * m_x * CCD_REAL(0.5),
                   CollisionGeometry::sign(dirRot.v[1]) * m_y * CCD_REAL(0.5),
                   CollisionGeometry::sign(dirRot.v[2]) * m_z * CCD_REAL(0.5));
        break;
    }
    case Type::Sphere: {
        ccd_real_t len = CCD_SQRT(ccdVec3Len2(&dirRot));
        ccd_real_t scale = m_radius / len;

        ccdVec3Copy(&vert, &dirRot);
        ccdVec3Scale(&vert, scale);

        break;
    }
    case Type::Cylinder: {
        ccd_real_t xyShadow = CCD_SQRT(dirRot.v[0] * dirRot.v[0] + dirRot.v[1] * dirRot.v[1]);
        if (CCD_FABS(xyShadow) < 5 * CCD_EPS) // with double-precision built ccd, CCD_EPS is
                                              // DBL_EPSILON in float.h, around 2.22045e-16
        {
            ccdVec3Set(&vert, CCD_ZERO, CCD_ZERO, sign(dirRot.v[2]) * m_height * CCD_REAL(0.5));
        } else {
            ccd_real_t scale = m_radius / xyShadow;
            ccdVec3Set(&vert, scale * dirRot.v[0], scale * dirRot.v[1],
                       sign(dirRot.v[2]) * m_height * CCD_REAL(0.5));
        }
        break;
    }
    case Type::Capsule: {
        /*
            Consider the support vector "s", then the farthest point "p" along
            the capsule in direction of "s" is given as,

            p = [0, 0, sign(dot(s,[0,0,1]))] * L/2 + R * s

            Where L and R are the length/height and radius of the capsule,
            respectively.
        */
        ccd_real_t len = CCD_SQRT(dirRot.v[0] * dirRot.v[0] + dirRot.v[1] * dirRot.v[1] +
                                  dirRot.v[2] * dirRot.v[2]);
        ccd_real_t scale = m_radius / len;
        ccdVec3Set(&vert, scale * dirRot.v[0], scale * dirRot.v[1],
                   sign(dirRot.v[2]) * m_height * CCD_REAL(0.5) + scale * dirRot.v[2]);
        break;
    }
    case Type::ConvexMesh: {
        std::vector<ccd_real_t> dotProd;
        for (auto&& v : m_vertices) {
            dotProd.push_back(v.v[0] * dirRot.v[0] + v.v[1] * dirRot.v[1] + v.v[2] * dirRot.v[2]);
        }
        auto largestItemIter = std::max_element(dotProd.begin(), dotProd.end());
        std::size_t idx = largestItemIter - dotProd.begin();
        ccdVec3Set(&vert, m_vertices[idx].v[0], m_vertices[idx].v[1], m_vertices[idx].v[2]);
        break;
    }
    case Type::ConvexMeshFull:
    default: {
        // In hill climbing traversal, we can keep track of the indices of visited nodes in a vector
        // to avoid checking previously visited nodes, but it turns out there is not much
        // performance gain by doing this, and it is in fact quite costly if we try to recreate this
        // vector in every support call

        // std::vector<int> visited(m_numVertices, 0)

        ccd_real_t maxValue = m_vertices[0].v[0] * dirRot.v[0] + m_vertices[0].v[1] * dirRot.v[1] +
                              m_vertices[0].v[2] * dirRot.v[2];
        ccd_real_t val = maxValue;
        std::size_t currVertIdx = 0;
        std::size_t nextVertIdx = currVertIdx;

        int numIter = 0;
        do {
            currVertIdx = nextVertIdx;
            for (std::size_t k = 0; k < m_connectionList[currVertIdx].size(); k++) {
                std::size_t idx = static_cast<std::size_t>(m_connectionList[currVertIdx][k]);
                val = ccdVec3Dot(&m_vertices[idx], &dirRot);
                if (val > maxValue) {
                    maxValue = val;
                    nextVertIdx = m_connectionList[currVertIdx][k];
                }
                numIter++;
            }

        } while (currVertIdx != nextVertIdx);
        // std::cout << "numIter = " << numIter << std::endl
        ccdVec3Set(&vert, m_vertices[currVertIdx].v[0], m_vertices[currVertIdx].v[1],
                   m_vertices[currVertIdx].v[2]);
    }
    }

    // transform support point to world coordinates (quat * vert + pos)
    ccdQuatRotVec(&vert, &m_quat);
    ccdVec3Add(&vert, &m_pos);
    ccdVec3Copy(ccdSupportVertex, &vert);
}

///
void CollisionGeometry::constructConnectionList() {
    std::cout << "construct connection list" << std::endl;
    // initialize
    m_connectionList.resize(m_numVertices);
    for (std::size_t i = 0; i < m_numVertices; i++) {
        m_connectionList[i].resize(16);
        m_connectionList[i].clear();
    }

    // populate
    for (std::size_t j = 0; j < m_numFaces; j++) {
        m_connectionList[m_faces[j].v[0]].push_back(m_faces[j].v[1]);
        m_connectionList[m_faces[j].v[0]].push_back(m_faces[j].v[2]);
        m_connectionList[m_faces[j].v[1]].push_back(m_faces[j].v[0]);
        m_connectionList[m_faces[j].v[1]].push_back(m_faces[j].v[2]);
        m_connectionList[m_faces[j].v[2]].push_back(m_faces[j].v[0]);
        m_connectionList[m_faces[j].v[2]].push_back(m_faces[j].v[1]);
    }

    // reduce
    for (std::size_t i = 0; i < m_numVertices; i++) {
        std::sort(m_connectionList[i].begin(), m_connectionList[i].end());
        auto last = std::unique(m_connectionList[i].begin(), m_connectionList[i].end());
        m_connectionList[i].erase(last, m_connectionList[i].end());
    }
}


} // namespace shared_robotics
