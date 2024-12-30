#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
// #include <pinocchio/algorithm/coriolis.hpp>
#include <pinocchio/algorithm/rnea.hpp> // Для вычисления гравитационных сил
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <iostream>
#include <Eigen/Dense>

int main() {
    // Путь к вашему URDF-файлу
    const std::string urdf_filename = "../iiwa/iiwa.urdf";

    // Загружаем модель
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    // Создаем объект данных
    pinocchio::Data data(model);

    // Пример: задаем конфигурацию и скорости (q - обобщенные координаты, v - обобщенные скорости)
    Eigen::VectorXd q(7); // = Eigen::VectorXd::Zero(model.nq); // Конфигурация робота (например, все суставы в 0)
    Eigen::VectorXd v(7); // = Eigen::VectorXd::Zero(model.nv); // Скорости (например, робот неподвижен)
    q << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    v << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    // 1. Матрица инерции M(q)
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    Eigen::VectorXd g;

    clock_t t = clock();

    M = pinocchio::crba(model, data, q);
    // M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
    // std::cout << "Матрица инерции M(q):\n" << M << std::endl;

    // 2. Матрица Кориолисовых сил C(q, v)
    C = pinocchio::computeCoriolisMatrix(model, data, q, v);
    // std::cout << "Матрица Кориолисовых сил C(q, v):\n" << C << std::endl;

    // 3. Вектор гравитационных сил g(q)
    g = pinocchio::computeGeneralizedGravity(model, data, q);
    // std::cout << "Вектор гравитационных сил g(q):\n" << g.transpose() << std::endl;

    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    return 0;
}
