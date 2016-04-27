// Copyright (C) 2016 LAAS-CNRS
// Author: Anna Seppala 
//
// This file is part of the hpp-affordance.
//
// hpp-affordance is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-affordance is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-affordance.  If not, see <http://www.gnu.org/licenses/>.

#include <hpp/affordance/operations.hh>
#include <hpp/affordance/fwd.hh>

#define BOOST_TEST_MODULE test-operations
#include <boost/test/included/unit_test.hpp>

const double epsilon = 10e-6;

bool compDouble(const double a, const double b)
{
    return a-b < epsilon;
}

bool compVec(const fcl::Vec3f& a, const fcl::Vec3f& b)
{
    return (a-b).norm() < epsilon;
}

BOOST_AUTO_TEST_SUITE (test_affordance)

BOOST_AUTO_TEST_CASE (operations)
{
  hpp::affordance::SupportOperationPtr_t support (new hpp::affordance::SupportOperation(0.3));
  hpp::affordance::LeanOperationPtr_t lean (new hpp::affordance::LeanOperation(0.1));

  std::vector <hpp::affordance::OperationBasePtr_t> operations;
  operations.push_back(support);
  operations.push_back(lean);

  const fcl::Vec3f normal1(0, 0, 1);

    BOOST_CHECK_MESSAGE (compVec(support->zWorld_,normal1),
		"default value for zWorld should be " << normal1 << " but is " << support->zWorld_);

    BOOST_CHECK_MESSAGE (compDouble(lean->margin_,0.1),
		"margin should match the one given when creating operation");

	BOOST_CHECK_MESSAGE (operations.size () == 2, 
		"operation vector should have size 2 after adding support and lean");
}
BOOST_AUTO_TEST_SUITE_END ()
