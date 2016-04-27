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

#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <algorithm>


namespace hpp {
  namespace affordance {

    BVHModelOBConst_Ptr_t GetModel (const fcl::CollisionObjectConstPtr_t& object)
    {
        assert (object->collisionGeometry ()->getNodeType () == fcl::BV_OBBRSS);
        const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>
                                            (object->collisionGeometry ());
        assert (model->getModelType () == fcl::BVH_MODEL_TRIANGLES);
        return model;
    }

    void searchLinkedTriangles(std::vector<unsigned int>& listPotential, const OperationBasePtr_t& refOp,
                               const std::vector<Triangle>& allTris, std::vector<unsigned int>& searchableTris,
                               const unsigned int& refTriIdx, double& area)
    {
			// TODO: think of a better way of declaring margins?
      const double marginRad = 0.3;
      const double margin = 1e-15;
			const Triangle& refTri = allTris[refTriIdx];
			searchableTris.erase (std::remove(searchableTris.begin (),
				searchableTris.end (), refTriIdx), searchableTris.end ());
      for (unsigned int searchIdx = 0; searchIdx < allTris.size (); searchIdx++) {
        std::vector<unsigned int>::iterator it = std::find (searchableTris.begin (),
                                                            searchableTris.end (), searchIdx);
          if (it == searchableTris.end ()) {
            continue;
          }
          std::vector<fcl::Vec3f> refPoints;
          refPoints.push_back(refTri.points.p1);
          refPoints.push_back(refTri.points.p2);
          refPoints.push_back(refTri.points.p3);
        for (unsigned int vertIdx = 0; vertIdx < 3; vertIdx++) {
          const Triangle& searchTri = allTris [searchIdx];
          if ((refPoints[vertIdx] - searchTri.points.p1).sqrLength () < margin
              || (refPoints[vertIdx] - searchTri.points.p2).sqrLength () < margin
              || (refPoints[vertIdx] - searchTri.points.p3).sqrLength () < margin) {
            if (refOp->requirement (searchTri.normal)) {
              if ((searchTri.normal - refTri.normal).sqrLength () < marginRad) {
                area += searchTri.area;
                listPotential.push_back (searchIdx);
                searchLinkedTriangles (listPotential, refOp, allTris,
                                       searchableTris, searchIdx, area);
              }
            } else {
              // if linked face does not fulfil global requirement, discard
							searchableTris.erase(std::remove(searchableTris.begin(),
								searchableTris.end(), searchIdx), searchableTris.end());
            }
            break; // jump out of vertex loop if triangle already tested for affordance
          }
        }
      }
    }

    SemanticsDataPtr_t affordanceAnalysis (const fcl::CollisionObjectPtr_t& colObj,
                                           const OperationBases_t& opVec)
    {
      BVHModelOBConst_Ptr_t model =  GetModel (colObj);

      std::vector <Triangle> triangles;
      std::vector <unsigned int> unsetTriangles;
      double totArea = .0;
      std::vector<std::vector<unsigned int> > potentialAffordances (opVec.size ());
      SemanticsDataPtr_t foundAffordances(new SemanticsData());

      for(int i = 0; i < model->num_tris; ++i)
      {
        TrianglePoints tri;
        fcl::Triangle fcltri = model->tri_indices [i];
        tri.p1 = colObj->getRotation () *
          model->vertices [fcltri [0]] + colObj->getTranslation ();
        tri.p2 = colObj->getRotation () *
          model->vertices [fcltri [1]] + colObj->getTranslation ();
        tri.p3 = colObj->getRotation () *
          model->vertices [fcltri [2]] + colObj->getTranslation ();

        triangles.push_back (Triangle (tri));
        // save vector index of triangles and their quantity.
				unsetTriangles.push_back(i);
      }
      std::vector <unsigned int> unseenTriangles;
      for (unsigned int triIdx = 0; triIdx < triangles.size (); triIdx++) {
        // look for triangle in set of triangles not yet given an affordance:
        std::vector<unsigned int>::iterator it = std::find (unsetTriangles.begin (), unsetTriangles.end (), triIdx);
        if (it == unsetTriangles.end ()) {
          continue;
        }
        // set list of searchable (unseen) triangles equal to all triangles not yet given an affordance.
        unseenTriangles = unsetTriangles;
        for (unsigned int opIdx = 0; opIdx < opVec.size (); opIdx++) {
          if (opVec[opIdx]->requirement (triangles[triIdx].normal)) {
             totArea += triangles[triIdx].area;
             potentialAffordances[opIdx].push_back(triIdx);
             searchLinkedTriangles(potentialAffordances [opIdx], opVec[opIdx],
						 	 triangles, unseenTriangles, triIdx, totArea);
            if (totArea > opVec[opIdx]->minArea_) {
              // save totArea for further use as well?
              AffordancePtr_t aff(new Affordance (potentialAffordances [opIdx], colObj));
              foundAffordances->affordances_ [opIdx].push_back (aff);
              for (unsigned int removeIdx = 0; removeIdx < potentialAffordances [opIdx].size (); removeIdx++) {
                std::remove (unsetTriangles.begin (), unsetTriangles.end (), potentialAffordances [opIdx][removeIdx]);
                unsetTriangles.pop_back ();
              }
                // potentialAffordances [opIdx].clear ();
             }
             potentialAffordances [opIdx].clear ();
             totArea = .0;
             break;
          } else if (opIdx >= opVec.size () -1) {
              // delete triangle if it did not fulfil any requirements
              std::remove (unsetTriangles.begin (), unsetTriangles.end (), triIdx);
              unsetTriangles.pop_back ();
            }
        }
      }
      return foundAffordances;
    }

    std::vector<CollisionObjects_t> getAffordanceObjects
                                             (const SemanticsDataPtr_t& sData)
    {
      std::vector<CollisionObjects_t> affObjs;
			affObjs.clear();
      for (unsigned int affIdx = 0; affIdx < sData->affordances_.size (); affIdx ++) {
        std::vector<fcl::CollisionObjectPtr_t> objVec;
				objVec.clear();
        affObjs.push_back (objVec);
        // get number of affordances of specific type (lean OR support etc)
        // this corresponds to number of objects to be created for specific aff type
        long unsigned int len = sData->affordances_[affIdx].size ();
        for (unsigned int idx = 0; idx < len; idx++) {
          std::vector<fcl::Vec3f> vertices;
          std::vector<fcl::Triangle> triangles;
          hpp::affordance::AffordancePtr_t affPtr = sData->affordances_[affIdx][idx];
          BVHModelOBConst_Ptr_t model =  GetModel (affPtr->colObj_);
          for (unsigned int triIdx = 0; triIdx <  affPtr->indices_.size (); triIdx++) {
						// give triangles of new object new vertex indices (start from 0
						// and go up to 3*nbTris - 1 [all tris have 3 unique indices])
            triangles.push_back (fcl::Triangle ((3*triIdx),(1 + 3*triIdx),(2 + 3*triIdx)));
						for (unsigned int vertIdx = 0; vertIdx < 3; vertIdx++) {
							vertices.push_back (model->vertices [vertIdx + 3*(affPtr->indices_[triIdx])]);
						}
          }
          
          BVHModelOB_Ptr_t model1 (new BVHModelOB ());
          // add the mesh data into the BVHModel structure
          model1->beginModel ();
          model1->addSubModel (vertices, triangles);
          model1->endModel ();
          // create affordance collision object from created affModel and 
          // tranformation of corresponding reference collision object.
          fcl::CollisionObjectPtr_t obj (new fcl::CollisionObject(model1,
                                         affPtr->colObj_->getTransform ()));
          affObjs[affIdx].push_back (obj);
        }
      }
      return affObjs;
    }

		OperationBases_t createOperations ()
		{
			affordance::SupportOperationPtr_t support (new affordance::SupportOperation());
      affordance::LeanOperationPtr_t lean (new affordance::LeanOperation(0.1));

      affordance::OperationBases_t operations;
      operations.push_back(support);
      operations.push_back(lean);
			
			return operations;
		}

  } // namespace affordance
} // namespace hpp


