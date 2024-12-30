#include <iostream>

#include <time.h>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int /* argc */, char ** /* argv */)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel("../iiwa/iiwa.urdf", model);
  pinocchio::Data data(model);

  std::cout << "model name: " << model.name << std::endl;

  Eigen::Matrix<double,3,3> rotation;
  // rotation << 1, 0, 0,
  //             0, 1, 0,
  //             0, 0, 1;

  rotation << -0.350179,    0.936683, 7.58451e-05,
              0.597309,    0.223366,  -0.770279,
              -0.721524,    -0.26969,   -0.637707;

  const int JOINT_ID = 7;
  const pinocchio::SE3 oMdes(rotation, Eigen::Vector3d(0.0, -0.5, 0.45));

  Eigen::VectorXd q = pinocchio::neutral(model);
  const double eps = 1e-3;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-5;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);

  pinocchio::SE3 iMd;
  pinocchio::Data::Matrix6 Jlog;
  pinocchio::Data::Matrix6 JJt;

  clock_t t = clock();

  for (int i = 0;; i++)
  {
    pinocchio::forwardKinematics(model, data, q);
    iMd = data.oMi[JOINT_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector(); // in joint frame
    if (err.norm() < eps)
    {
      success = true;
      std::cout << i << std::endl;
      break;
    }
    if (i >= IT_MAX)
    {
      success = false;
      std::cout << i << std::endl;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
    // if (!(i % 10))
    //   std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  t = clock() - t;
  double time_taken = ((double)t)/CLOCKS_PER_SEC;
  std::cout << "Время расчета: " << time_taken*1000 << std::endl;

  if (success)
  {
    std::cout << "Convergence achieved!" << std::endl;
  }
  else
  {
    std::cout
      << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
      << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;

  pinocchio::Data data1(model);
  forwardKinematics(model, data1, q);

  std::ofstream ofs;

  ofs.open("../Position.txt", std::ofstream::out);

  if (!ofs.is_open()) { 
        std::cout << "ERROR" << std::endl;
  } 

  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) 
  {
    ofs << std::setprecision(3) << data1.oMi[joint_id].translation().transpose()[0] << "\t" 
                                << data1.oMi[joint_id].translation().transpose()[1] << "\t" 
                                << data1.oMi[joint_id].translation().transpose()[2] << std::endl;
  }
}