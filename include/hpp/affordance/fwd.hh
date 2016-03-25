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

#ifndef HPP_AFFORDANCE_FWD_HH
#define HPP_AFFORDANCE_FWD_HH

#include <vector>
#include <map>
#include <hpp/util/pointer.hh>
#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/math/vec_3f.h>

namespace hpp {
  namespace affordance {
    HPP_PREDEF_CLASS (AffordanceExtraction);
    HPP_PREDEF_CLASS (SemanticsData);
    HPP_PREDEF_CLASS (Affordance);
    HPP_PREDEF_CLASS (OperationBase);
    HPP_PREDEF_CLASS (SupportOperation);
    HPP_PREDEF_CLASS (LeanOperation);

    typedef fcl::BVHModel<fcl::OBBRSS> BVHModelOB;
    typedef boost::shared_ptr<BVHModelOB> BVHModelOB_Ptr_t;
    typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;
    typedef boost::shared_ptr <SupportOperation> SupportOperationPtr_t;
    typedef boost::shared_ptr <LeanOperation> LeanOperationPtr_t;
    typedef boost::shared_ptr <fcl::CollisionObject> CollisionObjectPtr_t;
    typedef std::vector<CollisionObjectPtr_t> CollisionObjects_t;
    typedef boost::shared_ptr <OperationBase> OperationBasePtr_t; 
    typedef std::vector<OperationBasePtr_t> OperationBases_t;
    typedef boost::shared_ptr <SemanticsData> SemanticsDataPtr_t;
    typedef boost::shared_ptr <Affordance> AffordancePtr_t;
    typedef boost::shared_ptr <AffordanceExtraction> AffordanceExtractionPtr_t;
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_FWD_HH

