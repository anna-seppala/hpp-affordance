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

		struct TrianglePoints
    {
      fcl::Vec3f p1, p2, p3;
    };
		// helper class to save triangle information
    struct Triangle
    {
      Triangle () {}

      Triangle (const TrianglePoints& inPoints):
                points (inPoints)
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
    };

		// helper function to extract mesh model of an fcl::collisionObstacle
    BVHModelOBConst_Ptr_t GetModel (const fcl::CollisionObjectConstPtr_t& object);

    /// \addtogroup affordance
    /// \{
		/// Class that saves a reference collision object and indices to
		/// those of its mesh triangles that form one affordance object.
		/// This information will later be used to create fcl::collisionObjects
		/// for each individual affordance.
    class Affordance
    {
      public:

      Affordance () {}
			/// Constructor for an Affordance object
			///
			/// \param idxVec vector of triangle indices corresponding to
			///        the mesh model within the reference fcl::collisionObject
			/// \param colObj reference to pointer to fcl::collisionObject
			///        containing the found affordance objects.
      Affordance (const std::vector<unsigned int>& idxVec,
                  const fcl::CollisionObjectPtr_t& colObj):
                  indices_(idxVec), colObj_(colObj) {}
      std::vector<unsigned int> indices_;
      fcl::CollisionObjectPtr_t colObj_;
    };
		///	Class containing a vector of vectors of Affordance objects.
		/// The length of the outer vector corresponds to the amount of different
		/// affordance types found, and that of the inner vector to the amount
		/// of affordance objects found for each affordance type.
    class SemanticsData
    {
      public:

      SemanticsData () {}
      std::vector<std::vector<AffordancePtr_t> > affordances_;
			private:
			SemanticsData(const SemanticsData&); // Prevent copy-construction
			SemanticsData& operator=(const SemanticsData&);
    };


    /// Free function that searches through a vector of mesh triangles
    /// and saves the triangle indices that form a potential affordance
		/// object. Given a reference triangle index that fullfils the affordance
		/// requirements of a given affordance type, the function recursively goes 
		/// all triangles that ara linked to the the reference. Also saves the
		/// through total area of found potential affordance.
		///
		/// \param listPotential reference to the vector of triangle indices
		///				 that form one potential affordance object. At every recursive
		///        step maximum one triangle is added to the vector.
		/// \param refOp operation that determines which affordance type the
		/// 			 triangles will be tested for.
		/// \param allTris all triangles of a given fcl::collisionObject mesh.
		///				 This parameter is only used as to verify global (affordance type)
		/// 			 and local (neighbouring triangles) requirements, and is not 
		///				 modified within the function.
		/// \param searchableTris vector of triangle indices that should be tested
		/// 			 in the search for more triangles. Whenever a triangle is found,
		/// 			 it is deleted from the vector and will not be tested again.
		///				 Similarly, if a triangle is tested once and does not fullfil
		///				 the global requirement set by the affordance type, it is deleted
		/// 			 and will not be tested again in subsequent recursive steps.
		/// \param refTriIdx index corresponding to the last found triangle that
		///				 fullfils both the local and the global requirement. It is then
		///				 used as reference in the following recursive step.
		/// \param area total area of all triangles that are part of the potential
		///				 affordance object. Every time a triangle fulfilling all set
		///				 requirements is found, its are is added to the previous total
		///				 before going to the next recursive step.
    void searchLinkedTriangles(std::vector<unsigned int>& listPotential,
                              const OperationBasePtr_t& refOp,
                              const std::vector<Triangle>& allTris,
                              std::vector<unsigned int>& searchableTris,
                              const unsigned int& refTriIdx, double& area);

		/// Free function that extracts all affordances (of all types) from a given
		/// fcl::collisionObject.
		///
		/// \param colObj reference to a fcl::collisionObject pointer the triangles
		///				 of which will be searched for affordance objects.
		/// \param opVec vector of operation objects that determine which requirements
		///				 are set for which affordance type. The length of this vector
		///				 corresponds to the amount of different affordance types considered.
    SemanticsDataPtr_t affordanceAnalysis (const fcl::CollisionObjectPtr_t& colObj,
                                           const OperationBases_t & opVec);

		/// Free function that, given a semanticsData pointer, creates one
		/// fcl::collisionObject for every Affordance object.
		///
		/// \param sData reference to all found Affordance objects.
    std::vector<CollisionObjects_t> getAffordanceObjects
                                             (const SemanticsDataPtr_t& sData);

		/// Free helper function that creates a vector of operation objects.
		OperationBases_t createOperations ();

    /// \}
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_AFFORDANCE_EXTRACTION_HH
