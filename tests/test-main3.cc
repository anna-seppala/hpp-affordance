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
  
  std::vector <std::pair <const char*, hpp::affordance::OperationBasePtr_t> > operations;
  operations.push_back(std::make_pair("Support", support));
  operations.push_back(std::make_pair("Lean", lean));  

  hpp::affordance::AffordanceExtractionPtr_t u = hpp::affordance::AffordanceExtraction::create(operations);
  std::cout << "Affordance Extraction object created." << std::endl;

  std::cout << "z_world: " << support->getZWorld () << std::endl;
  std::cout << "margin1: " << support->getMargin () << std::endl;
  std::cout << "margin2: " << lean->getMargin () << std::endl;



  fcl::Vec3f normal(0, 0, 1);
  
   
  
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
