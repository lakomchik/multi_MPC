#ifndef MPC_H
#define MPC_H

#include <iostream>

#include <iomanip>

#include <iostream>

#include <vector>

# define PI 3.14159265358979323846
// autodiff include
#include <autodiff/forward/real.hpp>

#include <autodiff/forward/real/eigen.hpp>

#include <Eigen/Dense>

#include <nlopt.hpp>
 //#include "/usr/local/include/matplotlibcpp.h"

namespace MPC {
  const int N = 5; //Prediction horizon
  const int n_states = 4;
  const int n_controls = 1;
  const double dt = 0.025;
  const autodiff::real C1 = 4.3188;
  const autodiff::real C2 = 1.0479;
  const autodiff::real C3 = 22.0752;
  const autodiff::real C4 = 2.8561;

  //Eigen::DiagonalMatrix <autodiff::real, 1> const R (0.5);
  //Eigen::DiagonalMatrix <double, n_states> const Q (0.,0.5,5.5,0.1);
  class MPC {
    private:

      public:
      void set_q();
    void euler_integration(Eigen::Matrix < autodiff::real, n_states, 1 >
      const & curr_pos, Eigen::Matrix < autodiff::real, n_controls, 1 >
      const & curr_controls,
      Eigen::Matrix < autodiff::real, n_states, 1 > & next_pos, double time_step);

    Eigen::Matrix < autodiff::real, 1, 1 > calculate_objective(const Eigen::Matrix < autodiff::real, N * n_controls, 1 > & opt_var);

    Eigen::Matrix < autodiff::real, 1, 1 > multiple_shooting_objective(const Eigen::Matrix < autodiff::real, N * n_controls + (N + 1) * n_states, 1 > & opt_var);

    Eigen::Matrix < autodiff::real, n_states * (N + 1), 1 > multiple_constraints(const Eigen::Matrix < autodiff::real, N * n_controls + (N + 1) * n_states, 1 > & opt_var);

    void set_init_cond(Eigen::Matrix < autodiff::real, n_states, 1 >
      const & init_cond);

    void get_jacobian(Eigen::Matrix < autodiff::real, N * n_controls + (N + 1) * n_states, 1 > & vars,
      double & result);

    void get_constraints_jacobian(Eigen::Matrix < autodiff::real, N * n_controls + (N + 1) * n_states, 1 > vars,
      Eigen::Matrix < double, (N + 1) * n_states, 1 > & results);

    const Eigen::Matrix < autodiff::real, n_states, N + 1 > get_predicted_states();

    void set_reference(const Eigen::Matrix < autodiff::real, n_states, N > & ref_path,
      const Eigen::Matrix < autodiff::real, n_controls, N > & ref_con);
    void init_nlopt();
    double myfunc(const std::vector < double > & x, std::vector < double > & grad);
    void multi_constraint(unsigned m, double * result, unsigned n,
      const double * x, double * grad);

    Eigen::Matrix < double, n_controls, 1 > solve(Eigen::Matrix < double, n_states, 1 > init_cond, double & minf);
    Eigen::Matrix < double, n_controls, 1 > solve(Eigen::Matrix < double, n_states, 1 > init_cond, std::vector < double > & predicted_states);

    Eigen::Matrix < double, 1, N * n_controls + (N + 1) * n_states > func_Jacobian;
    Eigen::Matrix < double, (N + 1) * n_states, N * n_controls + (N + 1) * n_states > constr_Jacobian;
    Eigen::Matrix < autodiff::real, n_states, 1 > init_cond;
    Eigen::Matrix < autodiff::real, n_states, N + 1 > states;
    Eigen::Matrix < autodiff::real, n_states, N > reference_path;
    Eigen::Matrix < autodiff::real, n_controls, N > reference_controls;
    Eigen::Matrix < autodiff::real, n_states, n_states > Q;
    Eigen::Matrix < autodiff::real, n_controls, n_controls > R;

    nlopt::opt opt;
    std::vector < double > x;
    const double mb = 10;
    const double Ib = 1.40833;
    const double r = 0.2;
    const double l = 0.4;
    const double mw = 1.;
    const double g = 9.81;
    const double Iw = 0.02;

  };

}

#endif