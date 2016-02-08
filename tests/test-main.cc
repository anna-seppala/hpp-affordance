#include <iostream>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/fcl/data_types.h>
#include<hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>

int main ()
{
  hpp::affordance::AffordanceExtractionPtr_t u = hpp::affordance::AffordanceExtraction::create();
  std::cout << "Affordance Extraction object created." << std::endl;

  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;
  
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
  
  fcl::CollisionObject* obj = new fcl::CollisionObject(model, boxPose);

//  for (unsigned int i = 0; i < model->num_tris; i++) {
//    
//    
//  }

  return 0;
}
