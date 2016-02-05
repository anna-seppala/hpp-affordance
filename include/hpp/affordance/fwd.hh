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
# define HPP_AFFORDANCE_FWD_HH


//# include <vector>
//# include <deque>
//# include <list>
//# include <set>
//# include <hpp/util/pointer.hh>
//# include <hpp/constraints/fwd.hh>

namespace hpp {
  namespace affordance {
    HPP_PREDEF_CLASS (AffordanceExtraction);

    typedef boost::shared_ptr <AffordanceExtraction> AffordanceExtractionPtr_t;
  } // namespace affordance
} // namespace hpp

#endif // HPP_AFFORDANCE_FWD_HH

