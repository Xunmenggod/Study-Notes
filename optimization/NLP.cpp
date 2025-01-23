#include <math.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include "OsqpEigen/OsqpEigen.h"

using namespace Eigen;
using namespace std;

// @TODO: Eigen sparse matrix could only use insert instead of block
bool constructHessianNGradient(SparseMatrix<double> &hessian, VectorXd &gradient, const int K)
{
    int nx = 1 + K * 24;
    if (gradient.rows() != nx)
    {
        cout << "Gradient dimension mismatches with hessian!!!"
             << "\n nx: " << nx << " gradient rows: " << gradient.rows() << endl;
        return false;
    }

    hessian.resize(1 + K * 24, 1 + K * 24);
    MatrixXd Q = MatrixXd::Identity(6, 6);
    for (int i = 0; i < K; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float value = Q(j, j);
            hessian.insert(1 + i * 24 + 12 + j, 1 + i * 24 + 12 + j) = value;
            hessian.insert(1 + i * 24 + 18 + j, 1 + i * 24 + 18 + j) = value;
        }
        // hessian.block<6, 6>(1 + i * 24 + 12, 1 + i * 24 + 12) = MatrixXd::Identity(6, 6);
        // hessian.block<6, 6>(1 + i * 24 + 18, 1 + i * 24 + 18) = MatrixXd::Identity(6, 6);
    }
    // 1 / tf
    gradient[0] = 1;
    return true;
}

bool constructConstraints(SparseMatrix<double> &Ac, VectorXd &lb, VectorXd &ub, const int K,
                          const float max_tf)
{
    Ac.resize(1 + 2 * K * 24, 1 + K * 24);
    ub[0] = 1 / max_tf;
    float delta_t = max_tf / K;
    Ac.insert(0, 0) = 1;

    // Discritized A and B
    MatrixXd A = MatrixXd::Zero(18, 18);
    A.block<6, 6>(0, 6) = MatrixXd::Identity(6, 6);
    A.block<6, 6>(6, 12) = MatrixXd::Identity(6, 6);
    A = (A + MatrixXd::Identity(18, 18)) * delta_t;
    MatrixXd B = MatrixXd::Zero(18, 6);
    B.block<6, 6>(12, 0) = MatrixXd::Identity(6, 6);
    B = B * delta_t;
    // first vel acc jerk constrained to zero
    for (int i = 0; i < 18; i++)
        Ac.insert(1 + 6 + i, 1 + 6 + i) = 1;
    // Constrain the initial position inside the workspace
    // lb.block<6, 1>(1, 0) << -0.6, -0.6, -0.6, -0.6, -0.6, -0.6;
    // ub.block<6, 1>(1, 0) << 0.6, 0.6, 0.6, 0.6, 0.6, 0.6;

    for (int i = 1; i < K; i++)
    {
        // first nx * 24 is about the model constraint, x_k+1 = x+k + h*k
        for (int j = 0; j < 18; j++)
        {
            for (int k = 0; k < 18; k++)
            {
                Ac.insert(i * 24 + 1 + j, (i - 1) * 24 + 1 + k) = A(j, k);
                // cout << "A i j & k: " << i << "," << j << "," << k << endl;
            }
            for (int k = 0; k < 6; k++)
            {
                // cout << "B j & k: " << j << k << endl;
                Ac.insert(i * 24 + 1 + j, (i - 1) * 24 + 18 + 1 + k) = B(j, k);
            }
            Ac.insert(i * 24 + 1 + j, i * 24 + 1 + j) = -1;
        }
        // Ac.block<18, 18>(i * 24 + 1, (i - 1) * 24 + 1) = A * delta_t;
        // Ac.block<18, 6>(i * 24 + 18 + 1, (i - 1) * 24 + 18 + 1) = B * delta_t;
        // Ac.block<18, 18>(i * 24 + 1, i * 24 + 1) = -MatrixXd::Identity(18, 18);
    }

    for (int i = 0; i < K; i++)
    {
        // second nx is about the lb and upper bound ref, pos:{-2,2} vel:{0,10}, acc:{-50, 50},
        // jerk:{-1000,1000}
        for (int j = 0; j < 24; j++)
            Ac.insert(K * 24 + i * 24 + 1 + j, 1 + j + i * 24) = 1;
        // Ac.block<24, 24>(K * 24 + i * 24 + 1, 1 + i * 24) = MatrixXd::Identity(24, 24);
        lb.block<6, 1>(K * 24 + i * 24 + 1, 0) = -0.6 * VectorXd::Ones(6);
        lb.block<6, 1>(K * 24 + i * 24 + 12 + 1, 0) = -50 * VectorXd::Ones(6);
        lb.block<6, 1>(K * 24 + i * 24 + 18 + 1, 0) = -1000 * VectorXd::Ones(6);
        ub.block<6, 1>(K * 24 + i * 24 + 1, 0) = 0.6 * VectorXd::Ones(6);
        ub.block<6, 1>(K * 24 + i * 24 + 6 + 1, 0) = 15 * VectorXd::Ones(6);
        ub.block<6, 1>(K * 24 + i * 24 + 12 + 1, 0) = 50 * VectorXd::Ones(6);
        ub.block<6, 1>(K * 24 + i * 24 + 18 + 1, 0) = 1000 * VectorXd::Ones(6);
    }

    // final position which is about the target
    lb.block<6, 1>((2 * K - 1) * 24 + 1, 0) << 0, -0.5, 0.45, M_PI / 2, 0, 0;
    ub.block<6, 1>((2 * K - 1) * 24 + 1, 0) << 0, -0.5, 0.45, M_PI / 2, 0, 0;

    return true;
}

int main()
{
    // Matrix4d a;
    // a << 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4;
    // Matrix4d b = a.cwiseSqrt();
    // cout << a << endl << b << endl;
    // '''  End effector trajectory optimization x_o, state_x_k, u_k = J = argmin(
    //     1 / tf + In(-a ^ 2 + j ^ 2)) s.t.dot_[x, x_dot, x_ddot] =
    //     [ 0, 1, 0; 0, 0, 1; 0, 0, 0 ][x, x_dot, x_ddot] + [ 0, 0, 1 ][x + dddot] x_dot = x_ddot =
    //         x_dddot = 0, x_k = x_ref &l < x < u
    // '''
    const float max_tf = 0.45; // in seonds
    const int K = 15;
    // 24 means 4x6 -> pos, vel, acc, jerk for 6aixs
    // nc is 2k*24 is because of the equality and inequaility constraint
    // equality constraint is to enforce the dynamic model
    // inequality is about the lb and ub clipping
    int nx = 1 + K * 24, nu = K * 6, nc = 2 * K * 24 + 1;

    SparseMatrix<double> P;
    VectorXd q = VectorXd::Zero(nx);
    SparseMatrix<double> Ac;
    VectorXd lb = VectorXd::Zero(nc);
    VectorXd ub = VectorXd::Zero(nc);

    // update parameters based on model
    if (!constructHessianNGradient(P, q, K))
    {
        cout << "Failed to parse hessian and gradient!!!" << endl;
        exit(0);
    }
    cout << "Cal hessian!!" << endl;
    cout << "Hessian: " << P << endl;
    cout << "Gradient: " << q << endl;
    if (!constructConstraints(Ac, lb, ub, K, max_tf))
    {
        cout << "Failed to parse constraints!!!" << endl;
        exit(0);
    }
    cout << "cal constraints!!" << endl;
    cout << "Ac: " << Ac << endl;
    cout << "lb: " << lb << endl;
    cout << "ub: " << ub << endl;
    // solver settings
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);

    // construct qp problem
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);

    if (!solver.data()->setHessianMatrix(P))
    {
        cout << "Failed to set hessian in qp!!!" << endl;
        exit(0);
    }
    cout << "Constructed hessian!!" << endl;
    if (!solver.data()->setGradient(q))
    {
        cout << "Failed to set gradient in qp!!!" << endl;
        exit(0);
    }
    cout << "Set gradient" << endl;
    if (!solver.data()->setLinearConstraintsMatrix(Ac))
    {
        cout << "Failed to constraint matrix in qp!!!" << endl;
        exit(0);
    }
    cout << "set Ac" << endl;
    if (!solver.data()->setLowerBound(lb))
    {
        cout << "Failed to set lb in qp!!!" << endl;
        exit(0);
    }
    if (!solver.data()->setUpperBound(ub))
    {
        cout << "Failed to set ub in qp!!!" << endl;
        exit(0);
    }

    // solve problem
    if (!solver.initSolver())
    {
        cout << "Failed to init solver!!!" << endl;
        exit(0);
    }
    VectorXd solution;
    // if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    // {
    //     cout << "Failed to solve problem!!!" << endl;
    //     exit(0);
    // }
    solution = solver.getSolution();

    // parse solution and plotting for visualization
    // ofstream file;
    // file.open("./data.txt", std::ios::out | std::ios::trunc);

    // if (!file.is_open())
    // {
    //     cout << "Failed to open the file!!!" << end;
    //     exit(0);
    // }

    // VectorXd trajectory = solution.block<K * 24, 1>(1, 0);
    // for (int i = 0; i < K; i++)
    // {
    //     // xyz vxvyvz axayaz jxjyjz
    //     file << trajectory[i * 24] << " " << trajectory[i * 24 + 1] << " " << trajectory[i * 24 +
    //     2]
    //          << " " << trajectory[i * 24 + 6] << " " << trajectory[i * 24 + 7] << " "
    //          << trajectory[i * 24 + 8] << " " << trajectory[i * 24 + 12] << " "
    //          << trajectory[i * 24 + 13] << " " << trajectory[i * 24 + 14] << " "
    //          << trajectory[i * 24 + 18] << " " << trajectory[i * 24 + 19] << " "
    //          << trajectory[i * 24 + 20] << endl;
    // }
    // cout << "Successfully writing the data to the txt file!" << endl;

    return 1;
}