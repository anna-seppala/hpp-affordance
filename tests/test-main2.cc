#include <iostream>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/fcl/data_types.h>
#include<hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>
#include <hpp/affordance/operations.hh>

using namespace fcl;

unsigned int rand_interval(unsigned int min, unsigned int max);
void eulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R);
void generateRandomTransforms(FCL_REAL extents[6], std::vector<Transform3f>& transforms, std::size_t n);
void generateEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n);

int main ()
{
  hpp::affordance::SupportOperationPtr_t support (new hpp::affordance::SupportOperation());
  hpp::affordance::LeanOperationPtr_t lean (new hpp::affordance::LeanOperation(0.1));
  
  std::vector <std::pair <const char*, hpp::affordance::OperationBasePtr_t> > operations;
  operations.push_back(std::make_pair("Support", support));
  operations.push_back(std::make_pair("Lean", lean)); 

  hpp::affordance::AffordanceExtractionPtr_t u = hpp::affordance::AffordanceExtraction::create(operations);
  std::cout << "Affordance Extraction object created." << std::endl;

  std::vector<CollisionObject*> query;

  generateEnvironmentsMesh(query, 20, 10);

  std::cout << "Environment mesh created" << std::endl;

  return 0;
}

double rand_interval(double min, double max)
{

    int r;
    const double range = 1 + max - min;
    const double maxRand = RAND_MAX;
    const double buckets = maxRand / range;
    const double limit = buckets * range;

    /* Create equal size buckets all in a row, then fire randomly towards
     * the buckets until you land in one of them. All buckets are equally
     * likely. If you land off the end of the line of buckets, try again. */
    do
    {
        r = std::rand ();
    } while (r >= limit);

    return min + (r / buckets);
}

void eulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R)
{
  FCL_REAL c1 = cos(a);
  FCL_REAL c2 = cos(b);
  FCL_REAL c3 = cos(c);
  FCL_REAL s1 = sin(a);
  FCL_REAL s2 = sin(b);
  FCL_REAL s3 = sin(c);

  R.setValue(c1 * c2, - c2 * s1, s2,
             c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
             s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3);
}

void generateRandomTransforms(double extents[6], std::vector<Transform3f>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);

    {
      Matrix3f R;
      eulerToMatrix(a, b, c, R);
      Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}


void generateEnvironmentsMesh(std::vector<CollisionObject*>& env, double env_scale, std::size_t n)
{
  double extents[] = {-env_scale, env_scale, -env_scale, env_scale, -env_scale, env_scale};
  std::vector<Transform3f> transforms;

  generateRandomTransforms(extents, transforms, n);
  Box box(5, 10, 20);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3f());
    env.push_back(new CollisionObject(boost::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Sphere sphere(30);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(boost::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }

  generateRandomTransforms(extents, transforms, n);
  Cylinder cylinder(10, 40);
  for(std::size_t i = 0; i < n; ++i)
  {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3f(), 16, 16);
    env.push_back(new CollisionObject(boost::shared_ptr<CollisionGeometry>(model), transforms[i]));
  }
}

