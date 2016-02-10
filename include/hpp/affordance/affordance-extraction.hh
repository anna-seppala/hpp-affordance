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

    /// Extract whole-body affordances from fcl
    ///
    class HPP_AFFORDANCE_DLLAPI AffordanceExtraction
    {
      public:
        AffordanceExtraction (){}
        AffordanceExtraction (std::vector <std::pair <const char*, 
                              OperationBasePtr_t> > & operationVec): 
                              operations_(operationVec) {}
        static AffordanceExtractionPtr_t create (std::vector <std::pair <const char*,
                                                 OperationBasePtr_t> > & operationVec);
        // determine best form of input parameter -> pure triangles/collisionObjects/etc?
        void giveAffordances (const CollisionObjectPtr_t& obstacle);

      private:
        std::vector <std::pair <const char*, OperationBasePtr_t> > operations_;
    }; // class AffordanceExtraction 
    /// \}
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_AFFORDANCE_EXTRACTION_HH
