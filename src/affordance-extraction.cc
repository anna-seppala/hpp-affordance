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
#include<hpp/fcl/collision_object.h> 
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>

namespace hpp {
  namespace affordance {

    typedef fcl::BVHModel<fcl::OBBRSS> BVHModelOB;
    typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;

    AffordanceExtractionPtr_t AffordanceExtraction::create (std::vector <OperationBasePtr_t> & operationVec)
    {
      AffordanceExtraction* ptr = new AffordanceExtraction (operationVec);
      return AffordanceExtractionPtr_t (ptr);
    }
    
    BVHModelOBConst_Ptr_t GetModel (const fcl::CollisionObjectConstPtr_t object)
    {
        assert (object->collisionGeometry ()->getNodeType () == fcl::BV_OBBRSS);
        const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB> 
                                            (object->collisionGeometry ());
        assert (model->getModelType () == fcl::BVH_MODEL_TRIANGLES);
        return model;
    }

    void AffordanceExtraction::extractAffordances (const fcl::CollisionObjectPtr_t& colObj)
    {
      BVHModelOBConst_Ptr_t model =  GetModel (colObj);
      std::vector <Triangle> triangles;

      for(unsigned int i = 0; i < model->num_tris; ++i)
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
      }
      std::vector<int> unsetTriangles, unseenTriangles;
      for (unsigned int j = 0; j < triangles.size (); j++) {
       for (unsigned int k = 0; k < operations_.size (); k++) {
         if (operations_[k]->requirement (triangles[k].normal)) {
           
           
          } 
         
        } 
 

      }
    }

    std::vector <OperationBasePtr_t> AffordanceExtraction::getOperations ()
    {
      return operations_;
    }

  } // namespace affordance
} // namespace hpp


