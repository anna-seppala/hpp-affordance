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

  hpp::affordance::AffordanceExtractionPtr_t u = hpp::affordance::AffordanceExtraction::create(operations);
  std::cout << "Affordance Extraction object created." << std::endl;

  std::cout << "z_world: " << support->getZWorld () << std::endl;
  std::cout << "margin1: " << support->getMargin () << " and name: " << support->getAffordanceName () << std::endl;
  std::cout << "margin2: " << lean->getMargin () << " and name: " << lean->getAffordanceName () << std::endl;
  


  fcl::Vec3f normal1(0, 0, 1);
  fcl::Vec3f normal2(0, 1, 0);

  for (unsigned int i = 0; i < u->getOperations ().size(); i++) {
   std::cout << "testing requirement." << std::endl;
   if (u->getOperations ()[i]->requirement(normal2)) {
     std::cout << "requirement fulfilled!" << std::endl;
    }
  } 
  
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

  return 0;
}
