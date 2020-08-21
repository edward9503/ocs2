/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {
namespace LinearAlgebra {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void makePsdEigenvalue(Eigen::MatrixXd& squareMatrix, double minEigenvalue) {
  assert(squareMatrix.rows() == squareMatrix.cols());

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(squareMatrix, Eigen::EigenvaluesOnly);
  Eigen::VectorXd lambda = eig.eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.size(); j++) {
    if (lambda(j) < minEigenvalue) {
      hasNegativeEigenValue = true;
      lambda(j) = minEigenvalue;
    }
  }

  if (hasNegativeEigenValue) {
    eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
    squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  } else {
    squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void makePsdCholesky(Eigen::MatrixXd& A, double minEigenvalue) {
  using scalar_t = typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar;
  using dense_matrix_t = Eigen::MatrixXd;
  using sparse_matrix_t = Eigen::SparseMatrix<scalar_t>;

  assert(A.rows() == A.cols());

  // set the minimum eigenvalue
  A.diagonal().array() -= minEigenvalue;

  // S P' (A + E) P S = L L'
  sparse_matrix_t squareMatrix = 0.5 * A.sparseView();
  A.transposeInPlace();
  squareMatrix += 0.5 * A.sparseView();
  Eigen::IncompleteCholesky<scalar_t> incompleteCholesky(squareMatrix);
  // P' A P = M M'
  sparse_matrix_t M = (incompleteCholesky.scalingS().asDiagonal().inverse() * incompleteCholesky.matrixL());
  // L L' = P M M' P'
  sparse_matrix_t LmTwisted;
  LmTwisted.template selfadjointView<Eigen::Lower>() =
      M.template selfadjointView<Eigen::Lower>().twistedBy(incompleteCholesky.permutationP());
  dense_matrix_t L = dense_matrix_t(LmTwisted);
  // A = L L'
  A = L * L.transpose();

  // correction for the minimum eigenvalue
  A.diagonal().array() += minEigenvalue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void computeConstraintProjection(const Eigen::MatrixXd& Dm, const Eigen::MatrixXd& RmInvUmUmT, Eigen::MatrixXd& DmDagger,
                                 Eigen::MatrixXd& DmDaggerTRmDmDaggerUUT, Eigen::MatrixXd& RmInvConstrainedUUT) {
  const auto numConstraints = Dm.rows();
  const auto numInputs = Dm.cols();

  // Constraint Projectors are based on the QR decomposition
  Eigen::HouseholderQR<Eigen::MatrixXd> QRof_RmInvUmUmTT_DmT(RmInvUmUmT.transpose() * Dm.transpose());

  Eigen::MatrixXd QRof_RmInvUmUmTT_DmT_Rc = QRof_RmInvUmUmTT_DmT.matrixQR().topRows(numConstraints).template triangularView<Eigen::Upper>();

  // Computes the inverse of Rc with an efficient in-place forward-backward substitution
  // Turns out that this is equal to the UUT decomposition of DmDagger^T * R * DmDagger after simplification
  DmDaggerTRmDmDaggerUUT.setIdentity(numConstraints, numConstraints);
  QRof_RmInvUmUmTT_DmT_Rc.template triangularView<Eigen::Upper>().solveInPlace(DmDaggerTRmDmDaggerUUT);

  Eigen::MatrixXd QRof_RmInvUmUmTT_DmT_Q = QRof_RmInvUmUmTT_DmT.householderQ();
  // Auto take reference to the column view here without making a temporary
  auto QRof_RmInvUmUmTT_DmT_Qc = QRof_RmInvUmUmTT_DmT_Q.leftCols(numConstraints);
  auto QRof_RmInvUmUmTT_DmT_Qu = QRof_RmInvUmUmTT_DmT_Q.rightCols(numInputs - numConstraints);

  // Compute Weighted Pseudo Inverse, brackets used to compute the smaller, right-side product first
  DmDagger.noalias() = RmInvUmUmT * (QRof_RmInvUmUmTT_DmT_Qc * DmDaggerTRmDmDaggerUUT.transpose());

  // Constraint input cost UUT decomposition
  RmInvConstrainedUUT.noalias() = RmInvUmUmT * QRof_RmInvUmUmTT_DmT_Qu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int rank(const Eigen::MatrixXd& A) {
  return A.colPivHouseholderQr().rank();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXcd eigenvalues(const Eigen::MatrixXd& A) {
  return A.eigenvalues();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd symmetricEigenvalues(const matrix_t& A) {
  return A.selfadjointView<Eigen::Lower>().eigenvalues();
}

}  // namespace LinearAlgebra
}  // namespace ocs2
