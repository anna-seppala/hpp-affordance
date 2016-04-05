//
// Copyright (c) 2016 CNRS
// Authors: Anna Seppala
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_AFFORDANCE_AFFORDANCE_EXTRACTION_HH
#define HPP_AFFORDANCE_AFFORDANCE_EXTRACTION_HH

#include <hpp/affordance/fwd.hh>
#include <hpp/affordance/config.hh>

namespace hpp {
  namespace affordance {
    /// \addtogroup
    /// \{

    struct TrianglePoints
    {
      fcl::Vec3f p1, p2, p3;
    };

    struct Triangle
    {
      Triangle () {}

      Triangle (const fcl::Triangle& inFclTri, const TrianglePoints& inPoints):
                points (inPoints), fclTri (new  fcl::Triangle (inFclTri))
      {
        TriangleArea (points);
        TriangleNormal (points);
      }
      void TriangleArea(TrianglePoints& tri)
      {
        double a, b, c;
        a = (tri.p1 - tri.p2).norm();
        b = (tri.p2 - tri.p3).norm();
        c = (tri.p3 - tri.p1).norm();
        double s = 0.5 * (a + b + c);
        area = sqrt(s * (s-a) * (s-b) * (s-c));
      }
      void TriangleNormal(TrianglePoints& tri)
      {
        normal = (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
        normal.normalize();
      }
      TrianglePoints points;
      double area;
      fcl::Vec3f normal;
      boost::shared_ptr <fcl::Triangle> fclTri;
    };

    class Affordance
    {
      public:

      Affordance () {}

      Affordance (const std::vector<unsigned int>& idxVec,
                  const fcl::CollisionObjectPtr_t& colObj):
                  indices_(idxVec), colObj_(colObj) {}
      std::vector<unsigned int> indices_;
      fcl::CollisionObjectPtr_t colObj_;
    };

    class SemanticsData
    {
      public:

      SemanticsData (): SemanticsData (0) {}
      SemanticsData (const long unsigned int& affordanceCount)
      {
        affordances_.resize(affordanceCount);
      }
      std::vector<std::vector<AffordancePtr_t> > affordances_;
    };

    BVHModelOBConst_Ptr_t GetModel (const fcl::CollisionObjectConstPtr_t object);

    /// Free functions to extract whole-body affordances from fcl
    ///
    void searchLinkedTriangles(std::vector<unsigned int>& listPotential,
                              const OperationBasePtr_t& refOp,
                              const std::vector<Triangle>& allTris,
                              std::vector<unsigned int>& searchableTris,
                              const unsigned int& refTriIdx, double& area);

    SemanticsDataPtr_t affordanceAnalysis (const fcl::CollisionObjectPtr_t& colObj,
                                           const OperationBases_t & opVec);

    std::vector<CollisionObjects_t> getAffordanceObjects
                                             (const SemanticsDataPtr_t& sData);
    /// \}
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_AFFORDANCE_EXTRACTION_HH
