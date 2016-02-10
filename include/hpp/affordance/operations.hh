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

#ifndef HPP_AFFORDANCE_OPERATIONS_HH
#define HPP_AFFORDANCE_OPERATIONS_HH

# include <hpp/fcl/math/vec_3f.h>

namespace hpp {
  namespace affordance {

    class OperationBase
    {
      public:
        explicit OperationBase (): zWorld_(0,0,1), margin_(0.3) {}
        explicit OperationBase (const float & margin): zWorld_(0,0,1), margin_(margin) {}

        virtual bool requirement (fcl::Vec3f normal) {}

        // return world z axis
        fcl::Vec3f getZWorld () 
        {
          return zWorld_;
        }
        // return margin used for comparisons
        float getMargin ()
        {
          return margin_;
        }
      private:
        fcl::Vec3f zWorld_;
        float margin_;
    }; // class OperationBase

    class SupportOperation : public OperationBase
    {
      public:
         explicit SupportOperation (): OperationBase () {}
         explicit SupportOperation (const float & margin): OperationBase(margin) {}

        bool requirement (fcl::Vec3f normal) 
      {
        if ((getZWorld () - normal).sqrLength() < getMargin ()) {
         return true;
        }
	return false;
      }
    }; // class SupportOperation
   
    class LeanOperation : public OperationBase
    {
      public:
        explicit LeanOperation (): OperationBase () {}
        explicit LeanOperation (const float & margin): OperationBase(margin) {}
        bool requirement (fcl::Vec3f normal) 
        {
          if (fabs (normal.dot(getZWorld ())) < getMargin ()) {
            return true;
          }
	  return false;
        }
    }; // class LeanOperation
   
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_OPERATIONS_HH
