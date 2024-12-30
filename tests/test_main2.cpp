#include <iostream>
#include <time.h>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own
// directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char ** argv)
{
  using namespace pinocchio;

  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel("../iiwa/iiwa.urdf", model);

  // Build a data frame associated with the model
  Data data(model);

  // Sample a random joint configuration, joint velocities and accelerations
  Eigen::VectorXd q = randomConfiguration(model);      // in rad for the UR5
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // in rad/s for the UR5
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // in rad/s² for the UR5

  clock_t t = clock();

  // Computes the inverse dynamics (RNEA) for all the joints of the robot
  Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

  t = clock() - t;
  double time_taken = ((double)t)/CLOCKS_PER_SEC;
  std::cout << "Время расчета: " << time_taken*1000 << std::endl;

  // Print out to the vector of joint torques (in N.m)
  std::cout << "Joint torques: " << data.tau.transpose() << std::endl;
  return 0;
}