#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

#include "drake/solvers/mathematical_program.h"

/*
Methods for extracting Q, c, Aeq, beq, Aineq, bineq matrices from
drake::solvers::MathematicalProgram (assuming it's a QP)

Add filename to "hdrs" in BUILD file to use this

TODO fix seg fault during tests - happens because start index is wrong (out of matrix range)
*/

//#define PRINT_EIGEN

namespace drake{
namespace solvers{

void print_eigen(const std::string name, const Eigen::MatrixXd& mat) {
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "", "");
  std::cout << "-----" << std::endl;
  if(mat.cols() == 1) {
    std::cout << name << "= [ " << mat.transpose().format(CleanFmt) << " ]" << std::endl;
  }
  else {
    std::cout << name << "= [\n" << mat.format(CleanFmt) << " ]" << std::endl;
  }
}

void check_condition_number(const Eigen::MatrixXd& mat) {
  Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "", "");
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(mat);
	if (eigensolver.info() != Eigen::Success){
		std::cout << "Failed to compute eigenvectors." << std::endl;
		return;
	}
  Eigen::VectorXd eigvalues = eigensolver.eigenvalues();
  std::cout << "condition number: " << eigvalues.maxCoeff()/eigvalues.minCoeff() << std::endl;
  // std::cout << "eigenvalues: " << eigvalues.transpose().format(CleanFmt) << std::endl;
}

// Add both LinearConstraints and LinearEqualityConstraints to gurobi
// Assume all matrices are double?
void PrintLinearConstraints(MathematicalProgram& prog) {
  // find total number of constraints to resize matrices
  int num_eq = 0;
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.constraint();
    num_eq += constraint->A().rows();
  }

  int num_ineq = 0;
  for (const auto& binding : prog.linear_constraints()) {
    const auto& constraint = binding.constraint();
    num_ineq += constraint->A().rows();
  }

  Eigen::MatrixXd Aeq_all = Eigen::MatrixXd(num_eq, 68);
  Eigen::VectorXd beq_all = Eigen::VectorXd(num_eq);
  Eigen::MatrixXd Aineq_all = Eigen::MatrixXd(num_ineq, 68);
  Eigen::VectorXd bineq_all = Eigen::VectorXd(num_ineq);

  int eq_idx = 0;
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.constraint();
    int start = prog.FindDecisionVariableIndex(binding.variables()(0));

    Eigen::MatrixXd Aeq = constraint->A();
    Eigen::VectorXd beq = constraint->lower_bound();

    Aeq_all.block(eq_idx, start, Aeq.rows(), Aeq.cols()) = Aeq;
    beq_all.segment(start, beq.size()) = beq;

    // std::cout << constraint->get_description() << std::endl;
    // std::cout << "Aeq: " << Aeq.rows() << " " << Aeq.cols() << " beq: " << beq.rows() << " " << beq.cols() << std::endl;
    // std::cout << "start: " << start << std::endl;

    eq_idx += Aeq.rows();
  }

  int ineq_idx = 0;
  for (const auto& binding : prog.linear_constraints()) {
    // binding.variables() is eigen vector of symbolic::Variable
    const auto& constraint = binding.constraint();
    int start = prog.FindDecisionVariableIndex(binding.variables()(0));

    Eigen::MatrixXd Aineq = constraint->A();
    Eigen::VectorXd lb = constraint->lower_bound();

    Aineq_all.block(ineq_idx, start, Aineq.rows(), Aineq.cols()) = Aineq;
    bineq_all.segment(ineq_idx, lb.size()) = lb;

    // std::cout << constraint->get_description() << std::endl;
    // std::cout << "Aineq: " << Aineq.rows() << " " << Aineq.cols() << " bineq: " << lb.rows() << " " << lb.cols() << std::endl;
    // std::cout << "start: " << start << std::endl;

    ineq_idx += Aineq.rows();
  }

#ifdef PRINT_EIGEN
  print_eigen("Aeq", Aeq_all);
  print_eigen("beq", Aeq_all);
  print_eigen("Aineq", Aineq_all);
  print_eigen("bineq", bineq_all);
#endif
}

void PrintCosts(const MathematicalProgram& prog) {
  int num_vars = prog.num_vars();
  Eigen::MatrixXd Qall = Eigen::MatrixXd(num_vars, num_vars);
  Eigen::VectorXd ball = Eigen::VectorXd(num_vars);

  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    const Eigen::MatrixXd& Q = constraint->Q();
    const Eigen::VectorXd& b = constraint->b();

    int start = prog.FindDecisionVariableIndex(binding.variables()(0));

    Qall.block(start, start, Q.rows(), Q.cols()) += Q;
    ball.segment(start, b.size()) += b;

    // std::cout << constraint->get_description() << std::endl;
    // std::cout << "Q: " << Q.rows() << " " << Q.cols() << " c: " << b.rows() << " " << b.cols() << std::endl;
    // std::cout << "start: " << start << std::endl;
  }

  // check_condition_number(Qall);

#ifdef PRINT_EIGEN
  print_eigen("Q", Qall);
  print_eigen("c", ball);
#endif
}

} //end namespace solvers
} //end namespace drake
