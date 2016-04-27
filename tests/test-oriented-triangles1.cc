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

#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>

#define BOOST_TEST_MODULE test-oriented-triangles1
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE (test_affordance)

BOOST_AUTO_TEST_CASE (oriented_triangles1)
{
	hpp::affordance::SupportOperationPtr_t support (new hpp::affordance::SupportOperation());
  hpp::affordance::LeanOperationPtr_t lean (new hpp::affordance::LeanOperation(0.1));

  std::vector <hpp::affordance::OperationBasePtr_t> operations;
  operations.push_back(support);
  operations.push_back(lean);

	std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;

  typedef fcl::BVHModel<fcl::OBBRSS> Model;

	fcl::Vec3f vert1 (0,0,0);
  fcl::Vec3f vert2 (1,0,0);
  fcl::Vec3f vert3 (0,1,0);
	vertices.push_back (vert1);
	vertices.push_back (vert2);
	vertices.push_back (vert3);

	int size_ = 8;
	int affFound = 0;
  int affNotFound = 0;
	fcl::Vec3f T (0,0,0);
	fcl::Vec3f axis (1,0,0);
	fcl::Matrix3f R;

	for (int i = 0; i < size_; ++i) {
	  boost::shared_ptr<Model> model (new Model ());
		fcl::Triangle tri (0,1,2);
		triangles.push_back (tri); 
	
		fcl::Quaternion3f quat;
		quat.fromAxisAngle(axis,3.1416*float(5*i)/180.0);
		quat.toRotation(R);
	
	  fcl::Transform3f pose (R, T);
	
		model->beginModel ();
		model->addSubModel (vertices, triangles);
		model->endModel ();
	
	  boost::shared_ptr <fcl::CollisionObject> obj (new fcl::CollisionObject(model, pose));
	
	  hpp::affordance::SemanticsDataPtr_t h = hpp::affordance::affordanceAnalysis (obj, operations);

		if (h->affordances_[0].size() == 1 && h->affordances_[1].size() == 0) {
			affFound += 1;
		} else {
			affNotFound += 1;
		}
}
	BOOST_CHECK_MESSAGE (affFound == 7 && affNotFound == 1, 
		"Strictly six support affordances should have been found and one rejected. Now "<< affFound << 
		"were found and "<< affNotFound << " rejected.");
}
BOOST_AUTO_TEST_SUITE_END ()
