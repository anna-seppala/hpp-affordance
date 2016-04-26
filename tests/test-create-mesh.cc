#include <iostream>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/fcl/data_types.h>
#include<hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>
#include <hpp/affordance/operations.hh>

int main ()
{

  hpp::affordance::SupportOperationPtr_t support (new hpp::affordance::SupportOperation());
  hpp::affordance::LeanOperationPtr_t lean (new hpp::affordance::LeanOperation(0.1));

  std::vector <hpp::affordance::OperationBasePtr_t> operations;
  operations.push_back(support);
  operations.push_back(lean);

  std::cout << "z_world: " << support->zWorld_ << std::endl;
  std::cout << "margin1: " << support->margin_ << " and name: " << support->affordance_ << std::endl;
  std::cout << "margin2: " << lean->margin_ << " and name: " << lean->affordance_ << std::endl;

  fcl::Vec3f normal1(0, 0, 1);
  fcl::Vec3f normal2(0, 1, 0);


  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  boost::shared_ptr<Model> model (new Model ());

  fcl::Box box (5, 10, 20);
  fcl::Sphere sphere (30);

  fcl::Matrix3f R;
  R.setIdentity();
  fcl::Vec3f T(0, 0, 0);

  fcl::Transform3f boxPose (R, T);

  fcl::generateBVHModel (*model, box, boxPose);

  std::cout << "Model has " << model->num_tris << " triangles and "
            << model->num_vertices << " vertices." << std::endl;

  boost::shared_ptr <fcl::CollisionObject> obj (new fcl::CollisionObject(model, boxPose));

  hpp::affordance::SemanticsDataPtr_t h = hpp::affordance::affordanceAnalysis (obj, operations);


  std::vector<std::vector<boost::shared_ptr<Model> > > affModels;
  for (unsigned int affIdx = 0; affIdx < h->affordances_.size (); affIdx ++) {
    std::vector<boost::shared_ptr<Model> > modelVec;
    affModels.push_back (modelVec);
    // get number of affordances of specific type (lean OR support etc)
    // this corresponds to number of objects to be created for specific aff type
    long unsigned int len = h->affordances_[affIdx].size ();
    for (unsigned int idx = 0; idx < len; idx++) {
      std::vector<fcl::Vec3f> vertices;
      std::vector<fcl::Triangle> triangles;
      hpp::affordance::AffordancePtr_t affPtr = h->affordances_[affIdx][idx];
      for (unsigned int triIdx = 0; triIdx <  affPtr->indices_.size (); triIdx++) {
        vertices.push_back (model->vertices [affPtr->indices_[triIdx]]);
        triangles.push_back (model->tri_indices [affPtr->indices_[triIdx]]);
      }
      
      boost::shared_ptr<Model> model1 (new Model ());
      // add the mesh data into the BVHModel structure
      model1->beginModel ();
      model1->addSubModel (vertices, triangles);
      model1->endModel ();
      affModels[affIdx].push_back (model1);
    }
  }
  std::cout << "affordances saved to models" << std::endl;
  return 0;
}
