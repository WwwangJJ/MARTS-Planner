#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
using namespace std;

namespace gcopter
{
    double gra_, vel, acc;

    inline void traj_J(int &pieceN, 
                       Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
                       Eigen::VectorXi &velNums, Eigen::VectorXi &accNums)
    {
        Eigen::Vector3d vec;
        double gra_ = 9.8;
        double len = 5.0;
        pieceN = 6;
        int velCols = 5;
        int accCols = 3;
        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);
        pts.col(0)(0) = 0.0;
        pts.col(0)(1) = 0.0;
        pts.col(0)(2) = 0.0;
        vel = sqrt(gra_ * len);
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(0) = vel * vec;
        velNums(0) = 0;
        // acc = gra_;
        // vec << 0.0, 0.0, 1.0;
        // cout << vec<< endl;
        // vec.normalize();
        // wayAccs.col(0) = acc * vec;
        // accNums(0) = 0;

        pts.col(1)(0) = 0.5 * len;
        pts.col(1)(1) = 0.0;
        pts.col(1)(2) = (1 - 0.5 * sqrt(3)) * len;
        vel = sqrt((sqrt(3) - 1) * gra_ * len);
        vec << sqrt(3), 0.0, 1.0;
        vec.normalize();
        vels.col(1) = vel * vec;
        velNums(1) = 1;
        acc = (sqrt(3) - 1) * gra_; // an
        vec << -1.0, 0.0, sqrt(3.0);
        vec.normalize();
        accs.col(0) = acc * vec;
        acc = 0.5 * gra_; // at
        vec << -sqrt(3), 0.0, -1.0;
        vec.normalize();
        accs.col(0) += acc * vec;
        accNums(0) = 1;

        pts.col(2)(0) = 0.5 * sqrt(3) * len;
        pts.col(2)(1) = 0.0;
        pts.col(2)(2) = 0.5 * len;
        vel = 0.0;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(2) = vel * vec;
        velNums(2) = 2;
        acc = 0.5 * sqrt(3) * gra_;
        vec << -1.0, 0.0, -sqrt(3.0);
        vec.normalize();
        accs.col(1) = acc * vec;
        accNums(1) = 2;

        pts.col(3)(0) = 0.5 * len;
        pts.col(3)(1) = 0.0;
        pts.col(3)(2) = (1 - 0.5 * sqrt(3)) * len;
        vel = sqrt((sqrt(3) - 1) * gra_ * len);
        vec << -sqrt(3.0), 0.0, -1.0;
        vec.normalize();
        vels.col(3) = vel * vec;
        velNums(3) = 3;
        acc = (sqrt(3) - 1) * gra_; // an
        vec << -1.0, 0.0, sqrt(3.0);
        vec.normalize();
        accs.col(2) = acc * vec;
        acc = 0.5 * gra_; // at
        vec << -sqrt(3.0), 0.0, -1.0;
        vec.normalize();
        accs.col(2) += acc * vec;
        accNums(2) = 3;

        pts.col(4)(0) = 0.0;
        pts.col(4)(1) = 0.0;
        pts.col(4)(2) = 0.0;
        vel = sqrt(gra_ * len);
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(4) = vel * vec;
        velNums(4) = 4;
        // acc = gra_;
        // vec << 0.0, 0.0, 1.0;
        // vec.normalize();
        // wayAccs.col(4) = acc * vec;
        // accNums(4) = 4;

        cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
        cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
        cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
        cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
        cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
        cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
        // cout << "Acc0\t" << wayAccs.col(0)(0) << "\t" << wayAccs.col(0)(1) << "\t" << wayAccs.col(0)(2)<< endl;
        cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
        cout << "Acc1\t" << accs.col(0)(0) << "\t" << accs.col(0)(1) << "\t" << accs.col(0)(2) << endl;
        cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
        cout << "Acc2\t" << accs.col(1)(0) << "\t" << accs.col(1)(1) << "\t" << accs.col(1)(2) << endl;
        cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
        cout << "Acc3\t" << accs.col(2)(0) << "\t" << accs.col(2)(1) << "\t" << accs.col(2)(2) << endl;
        cout << "Vel4\t" << vels.col(4)(0) << "\t" << vels.col(4)(1) << "\t" << vels.col(4)(2) << endl;
        // cout << "Acc4\t" << wayAccs.col(4)(0) << "\t" << wayAccs.col(4)(1) << "\t" << wayAccs.col(4)(2)<< endl;
    }

    inline void traj_circle(int &pieceN, 
                            Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
                            Eigen::VectorXi &velNums, Eigen::VectorXi &accNums)
    {
        Eigen::Vector3d vec;
        double gra_ = 9.8;
        double len = 1.0;
        pieceN = 4;
        int velCols = 3;
        int accCols = 3;
        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);
        pts.col(0)(0) = len + len * sqrt(2) * 0.5;
        pts.col(0)(1) = -len * sqrt(2) * 0.5;
        pts.col(0)(2) = 0.0;
        vel = 3.1;
        vec << sqrt(2), sqrt(2), 0.0;
        vec.normalize();
        vels.col(0) = vel * vec;
        velNums(0) = 0;
        acc = vel * vel / len;
        vec << -sqrt(2), sqrt(2), 0.0;
        cout << vec<< endl;
        vec.normalize();
        accs.col(0) = acc * vec;
        accNums(0) = 0;

        pts.col(1)(0) = 2 * len;
        pts.col(1)(1) = 0.0;
        pts.col(1)(2) = 0.0;
        vel = 3.1;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(1) = vel * vec;
        velNums(1) = 1;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(1) = acc * vec;
        accNums(1) = 1;

        pts.col(2)(0) = len + len * sqrt(2) * 0.5;
        pts.col(2)(1) = len * sqrt(2) * 0.5;
        pts.col(2)(2) = 0.0;
        vel = 3.1;
        vec << -sqrt(2), sqrt(2), 0.0;
        vec.normalize();
        vels.col(2) = vel * vec;
        velNums(2) = 2;
        acc = vel * vel / len;
        vec << -sqrt(2), -sqrt(2), 0.0;
        vec.normalize();
        accs.col(2) = acc * vec;
        accNums(2) = 2;

        cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
        cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
        cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
        cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
        cout << "Acc0\t" << accs.col(0)(0) << "\t" << accs.col(0)(1) << "\t" << accs.col(0)(2)<< endl;
        cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
        cout << "Acc1\t" << accs.col(1)(0) << "\t" << accs.col(1)(1) << "\t" << accs.col(1)(2) << endl;
        cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
        cout << "Acc2\t" << accs.col(2)(0) << "\t" << accs.col(2)(1) << "\t" << accs.col(2)(2) << endl;
    }

    // inline void traj_small_circle(int &pieceN, 
    //                         Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                         Eigen::VectorXi &velNums, Eigen::VectorXi &accNums,const double &init_z)
    // {
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 12;
    //     int velCols = 11;
    //     int accCols = 11;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = len + len * cos(theta);
    //     pts.col(0)(1) = -len * sin(theta);
    //     pts.col(0)(2) = init_z;
    //     vel = v;
    //     vec << sin(theta), cos(theta), 0.0;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;
    //     acc = vel * vel / len;
    //     vec << -cos(theta), sin(theta), 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(0) = acc * vec;
    //     accNums(0) = 0;

    //     pts.col(1)(0) = 2 * len;
    //     pts.col(1)(1) = 0.0;
    //     pts.col(1)(2) = init_z;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 1;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(1) = acc * vec;
    //     accNums(1) = 1;

    //     pts.col(2)(0) = len;
    //     pts.col(2)(1) = len;
    //     pts.col(2)(2) = init_z;
    //     vel = v;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 2;
    //     acc = vel * vel / len;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     accs.col(2) = acc * vec;
    //     accNums(2) = 2;

    //     pts.col(3)(0) = 0.0;
    //     pts.col(3)(1) = 0.0;
    //     pts.col(3)(2) = init_z;
    //     vel = v;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 3;
    //     acc = vel * vel / len;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(3) = acc * vec;
    //     accNums(3) = 3;

    //     pts.col(4)(0) = len;
    //     pts.col(4)(1) = -len;
    //     pts.col(4)(2) = init_z;
    //     vel = v;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(4) = vel * vec;
    //     velNums(4) = 4;
    //     acc = vel * vel / len;
    //     vec << 0.0, 1.0, 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(4) = acc * vec;
    //     accNums(4) = 4;

    //     pts.col(5)(0) = 2 * len;
    //     pts.col(5)(1) = 0.0;
    //     pts.col(5)(2) = init_z;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(5) = vel * vec;
    //     velNums(5) = 5;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(5) = acc * vec;
    //     accNums(5) = 5;

    //     pts.col(6)(0) = len;
    //     pts.col(6)(1) = len;
    //     pts.col(6)(2) = init_z;
    //     vel = v;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(6) = vel * vec;
    //     velNums(6) = 6;
    //     acc = vel * vel / len;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     accs.col(6) = acc * vec;
    //     accNums(6) = 6;

    //     pts.col(7)(0) = 0.0;
    //     pts.col(7)(1) = 0.0;
    //     pts.col(7)(2) = init_z;
    //     vel = v;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     vels.col(7) = vel * vec;
    //     velNums(7) = 7;
    //     acc = vel * vel / len;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(7) = acc * vec;
    //     accNums(7) = 7;

    //     pts.col(8)(0) = len;
    //     pts.col(8)(1) = -len;
    //     pts.col(8)(2) = init_z;
    //     vel = v;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(8) = vel * vec;
    //     velNums(8) = 8;
    //     acc = vel * vel / len;
    //     vec << 0.0, 1.0, 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(8) = acc * vec;
    //     accNums(8) = 8;

    //     pts.col(9)(0) = 2 * len;
    //     pts.col(9)(1) = 0.0;
    //     pts.col(9)(2) = init_z;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(9) = vel * vec;
    //     velNums(9) = 9;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(9) = acc * vec;
    //     accNums(9) = 9;

    //     pts.col(10)(0) = len + len * cos(theta);
    //     pts.col(10)(1) = len * sin(theta);
    //     pts.col(10)(2) = init_z;
    //     vel = v;
    //     vec << -sin(theta), cos(theta), 0.0;
    //     vec.normalize();
    //     vels.col(10) = vel * vec;
    //     velNums(10) = 10;
    //     acc = vel * vel / len;
    //     vec << -cos(theta), -sin(theta), 0.0;
    //     vec.normalize();
    //     accs.col(10) = acc * vec;
    //     accNums(10) = 10;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     cout << "Pts5\t" << pts.col(5)(0) << "\t" << pts.col(5)(1) << "\t" << pts.col(5)(2) << endl;
    //     cout << "Pts6\t" << pts.col(6)(0) << "\t" << pts.col(6)(1) << "\t" << pts.col(6)(2) << endl;
    //     cout << "Pts7\t" << pts.col(7)(0) << "\t" << pts.col(7)(1) << "\t" << pts.col(7)(2) << endl;
    //     cout << "Pts8\t" << pts.col(8)(0) << "\t" << pts.col(8)(1) << "\t" << pts.col(8)(2) << endl;
    //     cout << "Pts9\t" << pts.col(9)(0) << "\t" << pts.col(9)(1) << "\t" << pts.col(9)(2) << endl;
    //     cout << "Pts10\t" << pts.col(10)(0) << "\t" << pts.col(10)(1) << "\t" << pts.col(10)(2) << endl;

    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     cout << "Acc0\t" << accs.col(0)(0) << "\t" << accs.col(0)(1) << "\t" << accs.col(0)(2)<< endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     cout << "Acc1\t" << accs.col(1)(0) << "\t" << accs.col(1)(1) << "\t" << accs.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     cout << "Acc2\t" << accs.col(2)(0) << "\t" << accs.col(2)(1) << "\t" << accs.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    //     cout << "Acc3\t" << accs.col(3)(0) << "\t" << accs.col(3)(1) << "\t" << accs.col(3)(2)<< endl;
    //     cout << "Vel4\t" << vels.col(4)(0) << "\t" << vels.col(4)(1) << "\t" << vels.col(4)(2) << endl;
    //     cout << "Acc4\t" << accs.col(4)(0) << "\t" << accs.col(4)(1) << "\t" << accs.col(4)(2) << endl;
    //     cout << "Vel5\t" << vels.col(5)(0) << "\t" << vels.col(5)(1) << "\t" << vels.col(5)(2) << endl;
    //     cout << "Acc5\t" << accs.col(5)(0) << "\t" << accs.col(5)(1) << "\t" << accs.col(5)(2) << endl;
    //     cout << "Vel6\t" << vels.col(6)(0) << "\t" << vels.col(6)(1) << "\t" << vels.col(6)(2) << endl;
    //     cout << "Acc6\t" << accs.col(6)(0) << "\t" << accs.col(6)(1) << "\t" << accs.col(6)(2)<< endl;
    //     cout << "Vel7\t" << vels.col(7)(0) << "\t" << vels.col(7)(1) << "\t" << vels.col(7)(2) << endl;
    //     cout << "Acc7\t" << accs.col(7)(0) << "\t" << accs.col(7)(1) << "\t" << accs.col(7)(2) << endl;
    //     cout << "Vel8\t" << vels.col(8)(0) << "\t" << vels.col(8)(1) << "\t" << vels.col(8)(2) << endl;
    //     cout << "Acc8\t" << accs.col(8)(0) << "\t" << accs.col(8)(1) << "\t" << accs.col(8)(2) << endl;
    //     cout << "Vel9\t" << vels.col(9)(0) << "\t" << vels.col(9)(1) << "\t" << vels.col(9)(2) << endl;
    //     cout << "Acc9\t" << accs.col(9)(0) << "\t" << accs.col(9)(1) << "\t" << accs.col(9)(2)<< endl;
    //     cout << "Vel10\t" << vels.col(10)(0) << "\t" << vels.col(10)(1) << "\t" << vels.col(10)(2) << endl;
    //     cout << "Acc10\t" << accs.col(10)(0) << "\t" << accs.col(10)(1) << "\t" << accs.col(10)(2) << endl;
    // }


    //  inline void traj_small_circle(int &pieceN, 
    //                         Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                         Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    // {

    //     // PosOffset:                  [0.0, 0.0, 0.0]      # three circle

    //     // InitLoadPos:                [-1.75, 0.0, 0.6]     # three small circle

    //     // EndLoadPos:                [-1.75, 0.0, 0.6]     # three small circle

    //     // WeightT:                    1000.0             # three small circle

    //     // WeightE:                    0.3               #  three small circle

    //     // ChiVec:                     [1.0, 1.0, 10000.0, 100.0, 1.0, 1.0, 1000.0]   # pos, vel, collision, omg, tiltAngle, thrust, uniformforce small circle
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.2;
    //     pieceN = 12;
    //     int velCols = 11;
    //     int accCols = 11;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = len + len * cos(theta);
    //     pts.col(0)(1) = -len * sin(theta);
    //     pts.col(0)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << sin(theta), cos(theta), 0.0;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;
    //     acc = vel * vel / len;
    //     vec << -cos(theta), sin(theta), 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(0) = acc * vec;
    //     accNums(0) = 0;

    //     pts.col(1)(0) = 2 * len;
    //     pts.col(1)(1) = 0.0;
    //     pts.col(1)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 1;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(1) = acc * vec;
    //     accNums(1) = 1;

    //     pts.col(2)(0) = len;
    //     pts.col(2)(1) = len;
    //     pts.col(2)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 2;
    //     acc = vel * vel / len;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     accs.col(2) = acc * vec;
    //     accNums(2) = 2;

    //     pts.col(3)(0) = 0.0;
    //     pts.col(3)(1) = 0.0;
    //     pts.col(3)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 3;
    //     acc = vel * vel / len;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(3) = acc * vec;
    //     accNums(3) = 3;

    //     pts.col(4)(0) = len;
    //     pts.col(4)(1) = -len;
    //     pts.col(4)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(4) = vel * vec;
    //     velNums(4) = 4;
    //     acc = vel * vel / len;
    //     vec << 0.0, 1.0, 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(4) = acc * vec;
    //     accNums(4) = 4;

    //     pts.col(5)(0) = 2 * len;
    //     pts.col(5)(1) = 0.0;
    //     pts.col(5)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(5) = vel * vec;
    //     velNums(5) = 5;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(5) = acc * vec;
    //     accNums(5) = 5;

    //     pts.col(6)(0) = len;
    //     pts.col(6)(1) = len;
    //     pts.col(6)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(6) = vel * vec;
    //     velNums(6) = 6;
    //     acc = vel * vel / len;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     accs.col(6) = acc * vec;
    //     accNums(6) = 6;

    //     pts.col(7)(0) = 0.0;
    //     pts.col(7)(1) = 0.0;
    //     pts.col(7)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 0.0, -1.0, 0.0;
    //     vec.normalize();
    //     vels.col(7) = vel * vec;
    //     velNums(7) = 7;
    //     acc = vel * vel / len;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(7) = acc * vec;
    //     accNums(7) = 7;

    //     pts.col(8)(0) = len;
    //     pts.col(8)(1) = -len;
    //     pts.col(8)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(8) = vel * vec;
    //     velNums(8) = 8;
    //     acc = vel * vel / len;
    //     vec << 0.0, 1.0, 0.0;
    //     cout << vec<< endl;
    //     vec.normalize();
    //     accs.col(8) = acc * vec;
    //     accNums(8) = 8;

    //     pts.col(9)(0) = 2 * len;
    //     pts.col(9)(1) = 0.0;
    //     pts.col(9)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << 0.0, 1.0, 0.0;
    //     vec.normalize();
    //     vels.col(9) = vel * vec;
    //     velNums(9) = 9;
    //     acc = vel * vel / len; // an
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     accs.col(9) = acc * vec;
    //     accNums(9) = 9;

    //     pts.col(10)(0) = len + len * cos(theta);
    //     pts.col(10)(1) = len * sin(theta);
    //     pts.col(10)(2) = init_z + 0.0;
    //     vel = v;
    //     vec << -sin(theta), cos(theta), 0.0;
    //     vec.normalize();
    //     vels.col(10) = vel * vec;
    //     velNums(10) = 10;
    //     acc = vel * vel / len;
    //     vec << -cos(theta), -sin(theta), 0.0;
    //     vec.normalize();
    //     accs.col(10) = acc * vec;
    //     accNums(10) = 10;
    // }

    inline void traj_small_circle(int &pieceN,
                                  Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels, Eigen::Matrix3Xd &accs,
                                  Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    {

        // InitLoadPos:                [-3.0,0.0, 0.6]     # three small circle

        // EndLoadPos:                [-3.0, 0.0, 0.6]     # three small circle

        // WeightT:                    1000.0             # three small circle

        // WeightE:                    0.3               #  three small circle

        // ChiVec:                     [1.0, 1.0, 10000.0, 100.0, 1.0, 1.0, 1000.0]   # pos, vel, collision, omg, tiltAngle, thrust, uniformforce small circle

        Eigen::Vector3d vec;
        double gra_ = 9.8;
        double len = 1.5;
        double v = 3.20;
        pieceN = 14;
        int velCols = 13;
        int accCols = 11;
        double theta = M_PI / 4.0;
        double x_shift = 0.75 * sqrt(3), y_shift = 0.75;

        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);


        pts.col(0)(0) = posOffset(0) + len * cos(theta);
        pts.col(0)(1) = posOffset(1) - y_shift - len * sin(theta);
        pts.col(0)(2) = posOffset(2) + init_z;
        vel = v;
        vec << sin(theta), cos(theta), 0.0;
        vec.normalize();
        vels.col(0) = vel * vec;
        velNums(0) = 0;

        pts.col(1)(0) = posOffset(0) + len;
        pts.col(1)(1) = posOffset(1) - y_shift;
        pts.col(1)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(1) = vel * vec;
        velNums(1) = 1;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(0) = acc * vec;
        accNums(0) = 1;

        pts.col(2)(0) = posOffset(0);
        pts.col(2)(1) = posOffset(1) - y_shift + len;
        pts.col(2)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(2) = vel * vec;
        velNums(2) = 2;
        acc = vel * vel / len;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        accs.col(1) = acc * vec;
        accNums(1) = 2;

        pts.col(3)(0) = posOffset(0) - len;
        pts.col(3)(1) = posOffset(1) - y_shift;
        pts.col(3)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(3) = vel * vec;
        velNums(3) = 3;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(2) = acc * vec;
        accNums(2) = 3;

        pts.col(4)(0) = posOffset(0) + x_shift + len;
        pts.col(4)(1) = posOffset(1);
        pts.col(4)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(4) = vel * vec;
        velNums(4) = 4;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(3) = acc * vec;
        accNums(3) = 4;

        pts.col(5)(0) = posOffset(0) + x_shift;
        pts.col(5)(1) = posOffset(1) + len;
        pts.col(5)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(5) = vel * vec;
        velNums(5) = 5;
        acc = vel * vel / len;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        accs.col(4) = acc * vec;
        accNums(4) = 5;

        pts.col(6)(0) = posOffset(0) + x_shift - len;
        pts.col(6)(1) = posOffset(1);
        pts.col(6)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(6) = vel * vec;
        velNums(6) = 6;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(5) = acc * vec;
        accNums(5) = 6;

        pts.col(7)(0) = posOffset(0) + x_shift;
        pts.col(7)(1) = posOffset(1) - len;
        pts.col(7)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(7) = vel * vec;
        velNums(7) = 7;
        acc = vel * vel / len;
        vec << 0.0, 1.0, 0.0;
        cout << vec<< endl;
        vec.normalize();
        accs.col(6) = acc * vec;
        accNums(6) = 7;

        pts.col(8)(0) = posOffset(0) + x_shift + len;
        pts.col(8)(1) = posOffset(1);
        pts.col(8)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(8) = vel * vec;
        velNums(8) = 8;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(7) = acc * vec;
        accNums(7) = 8;

        pts.col(9)(0) = posOffset(0) - len;
        pts.col(9)(1) = posOffset(1) + y_shift;
        pts.col(9)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(9) = vel * vec;
        velNums(9) = 9;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(8) = acc * vec;
        accNums(8) = 9;

        pts.col(10)(0) = posOffset(0);
        pts.col(10)(1) = posOffset(1) + y_shift - len;
        pts.col(10)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(10) = vel * vec;
        velNums(10) = 10;
        acc = vel * vel / len;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        accs.col(9) = acc * vec;
        accNums(9) = 10;

        pts.col(11)(0) = posOffset(0) + len;
        pts.col(11)(1) = posOffset(1) + y_shift;
        pts.col(11)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(11) = vel * vec;
        velNums(11) = 11;
        acc = vel * vel / len;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(10) = acc * vec;
        accNums(10) = 11;

        pts.col(12)(0) = posOffset(0) + len * cos(theta);
        pts.col(12)(1) = posOffset(1) + y_shift + len * sin(theta);
        pts.col(12)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -sin(theta), cos(theta), 0.0;
        vec.normalize();
        vels.col(12) = vel * vec;
        velNums(12) = 12;
    }

    inline void traj_single_circle(int &pieceN,
                                  Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels, Eigen::Matrix3Xd &accs,
                                  Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    {

        // InitLoadPos:                [-3.0,0.0, 0.6]     # three small circle

        // EndLoadPos:                [-3.0, 0.0, 0.6]     # three small circle

        // WeightT:                    1000.0             # three small circle

        // WeightE:                    0.3               #  three small circle

        // ChiVec:                     [1.0, 1.0, 10000.0, 100.0, 1.0, 1.0, 1000.0]   # pos, vel, collision, omg, tiltAngle, thrust, uniformforce small circle

        Eigen::Vector3d vec;
        double gra_ = 9.8;
        double len = 1.5;
        double v = 3.20;
        pieceN = 16;
        int velCols = 15;
        int accCols = 13;
        double theta = M_PI / 4.0;
        double x_shift = 0.75 * sqrt(3) - 0.5;

        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);


        pts.col(0)(0) = posOffset(0) + x_shift + len * cos(theta);
        pts.col(0)(1) = posOffset(1) - len * sin(theta);
        pts.col(0)(2) = posOffset(2) + init_z;
        vel = v;
        vec << sin(theta), cos(theta), 0.0;
        vec.normalize();
        vels.col(0) = vel * vec;
        velNums(0) = 0;

        pts.col(1)(0) = posOffset(0) + x_shift + len;
        pts.col(1)(1) = posOffset(1);
        pts.col(1)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(1) = vel * vec;
        velNums(1) = 1;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(0) = acc * vec;
        accNums(0) = 1;

        pts.col(2)(0) = posOffset(0) + x_shift;
        pts.col(2)(1) = posOffset(1) + len;
        pts.col(2)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(2) = vel * vec;
        velNums(2) = 2;
        acc = vel * vel / len;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        accs.col(1) = acc * vec;
        accNums(1) = 2;

        pts.col(3)(0) = posOffset(0) + x_shift - len;
        pts.col(3)(1) = posOffset(1);
        pts.col(3)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(3) = vel * vec;
        velNums(3) = 3;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(2) = acc * vec;
        accNums(2) = 3;

        pts.col(4)(0) = posOffset(0) + x_shift;
        pts.col(4)(1) = posOffset(1) - len;
        pts.col(4)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(4) = vel * vec;
        velNums(4) = 4;
        acc = vel * vel / len;
        vec << 0.0, 1.0, 0.0;
        cout << vec<< endl;
        vec.normalize();
        accs.col(3) = acc * vec;
        accNums(3) = 4;

        pts.col(5)(0) = posOffset(0) + x_shift + len;
        pts.col(5)(1) = posOffset(1);
        pts.col(5)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(5) = vel * vec;
        velNums(5) = 5;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(4) = acc * vec;
        accNums(4) = 5;

        pts.col(6)(0) = posOffset(0) + x_shift;
        pts.col(6)(1) = posOffset(1) + len;
        pts.col(6)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(6) = vel * vec;
        velNums(6) = 6;
        acc = vel * vel / len;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        accs.col(5) = acc * vec;
        accNums(5) = 6;

        pts.col(7)(0) = posOffset(0) + x_shift - len;
        pts.col(7)(1) = posOffset(1);
        pts.col(7)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(7) = vel * vec;
        velNums(7) = 7;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(6) = acc * vec;
        accNums(6) = 7;

        pts.col(8)(0) = posOffset(0) + x_shift;
        pts.col(8)(1) = posOffset(1) - len;
        pts.col(8)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(8) = vel * vec;
        velNums(8) = 8;
        acc = vel * vel / len;
        vec << 0.0, 1.0, 0.0;
        cout << vec<< endl;
        vec.normalize();
        accs.col(7) = acc * vec;
        accNums(7) = 8;

        pts.col(9)(0) = posOffset(0) + x_shift + len;
        pts.col(9)(1) = posOffset(1);
        pts.col(9)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(9) = vel * vec;
        velNums(9) = 9;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(8) = acc * vec;
        accNums(8) = 9;


        pts.col(10)(0) = posOffset(0) + x_shift;
        pts.col(10)(1) = posOffset(1) + len;
        pts.col(10)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(10) = vel * vec;
        velNums(10) = 10;
        acc = vel * vel / len;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        accs.col(9) = acc * vec;
        accNums(9) = 10;

        pts.col(11)(0) = posOffset(0) + x_shift - len;
        pts.col(11)(1) = posOffset(1);
        pts.col(11)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, -1.0, 0.0;
        vec.normalize();
        vels.col(11) = vel * vec;
        velNums(11) = 11;
        acc = vel * vel / len;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(10) = acc * vec;
        accNums(10) = 11;

        pts.col(12)(0) = posOffset(0) + x_shift;
        pts.col(12)(1) = posOffset(1) - len;
        pts.col(12)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(12) = vel * vec;
        velNums(12) = 12;
        acc = vel * vel / len;
        vec << 0.0, 1.0, 0.0;
        cout << vec<< endl;
        vec.normalize();
        accs.col(11) = acc * vec;
        accNums(11) = 12;

        pts.col(13)(0) = posOffset(0) + x_shift + len;
        pts.col(13)(1) = posOffset(1);
        pts.col(13)(2) = posOffset(2) + init_z;
        vel = v;
        vec << 0.0, 1.0, 0.0;
        vec.normalize();
        vels.col(13) = vel * vec;
        velNums(13) = 13;
        acc = vel * vel / len; // an
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        accs.col(12) = acc * vec;
        accNums(12) = 13;

        pts.col(14)(0) = posOffset(0) + x_shift + len * cos(theta);
        pts.col(14)(1) = posOffset(1) + len * sin(theta);
        pts.col(14)(2) = posOffset(2) + init_z;
        vel = v;
        vec << -sin(theta), cos(theta), 0.0;
        vec.normalize();
        vels.col(14) = vel * vec;
        velNums(14) = 14;
    }


     inline void traj_line(int &pieceN,
                                  Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels, Eigen::Matrix3Xd &accs,
                                  Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    {
        pieceN = 1;
        int velCols = 0;
        int accCols = 0;

        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);
    }

    // inline void traj_high_speed(int &pieceN, 
    //                             Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                             Eigen::VectorXi &velNums, Eigen::VectorXi &accNums)
    // {
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 7;
    //     int velCols = 5;
    //     int accCols = 0;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = 2;
    //     pts.col(0)(1) = -2;
    //     pts.col(0)(2) = 1.0;
    //     vel = 7;
    //     vec << 1.0, 0.0, -0.3;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;
    //     // acc = vel * vel / 5.0;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(0) = acc * vec;
    //     // accNums(0) = 0;

    //     pts.col(1)(0) = 5.0;
    //     pts.col(1)(1) = 0.0;
    //     pts.col(1)(2) = 0.5;
    //     // vel = 5;
    //     // vec << 0.0, 1.0, -0.15;
    //     // vec.normalize();
    //     // vels.col(1) = vel * vec;
    //     // velNums(1) = 1;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(1) = acc * vec;
    //     // accNums(1) = 1;

    //     pts.col(2)(0) = 3;
    //     pts.col(2)(1) = 2;
    //     pts.col(2)(2) = 0.0;
    //     vel = 6;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 2;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(2) = acc * vec;
    //     // accNums(2) = 2;

    //     pts.col(3)(0) = 0.0;
    //     pts.col(3)(1) = 1.3;
    //     pts.col(3)(2) = 0.3;
    //     vel = 5;
    //     vec << -1.3, -1.0, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 3;

    //     pts.col(4)(0) = -3.5;
    //     pts.col(4)(1) = -1.2;
    //     pts.col(4)(2) = 0.0;
    //     vel = 4;
    //     vec << -3.5, -1.2, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 4;
    //     // acc = vel * vel / len;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(3) = acc * vec;
    //     // accNums(3) = 3;

    //     pts.col(5)(0) = -3.5;
    //     pts.col(5)(1) = -1.2;
    //     pts.col(5)(2) = 2.0;
    //     vel = 4;
    //     vec << 3.5, 1.2, 0.0;
    //     vec.normalize();
    //     vels.col(4) = vel * vec;
    //     velNums(4) = 5;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(4) = acc * vec;
    //     // accNums(4) = 4;

    //     // pts.col(5)(0) = 2 * len;
    //     // pts.col(5)(1) = 0.0;
    //     // pts.col(5)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, 1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(5) = vel * vec;
    //     // velNums(5) = 5;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(5) = acc * vec;
    //     // accNums(5) = 5;

    //     // pts.col(6)(0) = len;
    //     // pts.col(6)(1) = len;
    //     // pts.col(6)(2) = 0.0;
    //     // vel = v;
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(6) = vel * vec;
    //     // velNums(6) = 6;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(6) = acc * vec;
    //     // accNums(6) = 6;

    //     // pts.col(7)(0) = 0.0;
    //     // pts.col(7)(1) = 0.0;
    //     // pts.col(7)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(7) = vel * vec;
    //     // velNums(7) = 7;
    //     // acc = vel * vel / len;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(7) = acc * vec;
    //     // accNums(7) = 7;

    //     // pts.col(8)(0) = len;
    //     // pts.col(8)(1) = -len;
    //     // pts.col(8)(2) = 0.0;
    //     // vel = v;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(8) = vel * vec;
    //     // velNums(8) = 8;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(8) = acc * vec;
    //     // accNums(8) = 8;

    //     // pts.col(9)(0) = 2 * len;
    //     // pts.col(9)(1) = 0.0;
    //     // pts.col(9)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, 1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(9) = vel * vec;
    //     // velNums(9) = 9;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(9) = acc * vec;
    //     // accNums(9) = 9;

    //     // pts.col(10)(0) = len + len * cos(theta);
    //     // pts.col(10)(1) = len * sin(theta);
    //     // pts.col(10)(2) = 0.0;
    //     // vel = v;
    //     // vec << -sin(theta), cos(theta), 0.0;
    //     // vec.normalize();
    //     // vels.col(10) = vel * vec;
    //     // velNums(10) = 10;
    //     // acc = vel * vel / len;
    //     // vec << -cos(theta), -sin(theta), 0.0;
    //     // vec.normalize();
    //     // accs.col(10) = acc * vec;
    //     // accNums(10) = 10;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     // cout << "Pts5\t" << pts.col(5)(0) << "\t" << pts.col(5)(1) << "\t" << pts.col(5)(2) << endl;
    //     // cout << "Pts6\t" << pts.col(6)(0) << "\t" << pts.col(6)(1) << "\t" << pts.col(6)(2) << endl;
    //     // cout << "Pts7\t" << pts.col(7)(0) << "\t" << pts.col(7)(1) << "\t" << pts.col(7)(2) << endl;
    //     // cout << "Pts8\t" << pts.col(8)(0) << "\t" << pts.col(8)(1) << "\t" << pts.col(8)(2) << endl;
    //     // cout << "Pts9\t" << pts.col(9)(0) << "\t" << pts.col(9)(1) << "\t" << pts.col(9)(2) << endl;
    //     // cout << "Pts10\t" << pts.col(10)(0) << "\t" << pts.col(10)(1) << "\t" << pts.col(10)(2) << endl;

    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     // cout << "Acc0\t" << accs.col(0)(0) << "\t" << accs.col(0)(1) << "\t" << accs.col(0)(2)<< endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     // cout << "Acc1\t" << accs.col(1)(0) << "\t" << accs.col(1)(1) << "\t" << accs.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     // cout << "Acc2\t" << accs.col(2)(0) << "\t" << accs.col(2)(1) << "\t" << accs.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    //     // cout << "Acc3\t" << accs.col(3)(0) << "\t" << accs.col(3)(1) << "\t" << accs.col(3)(2)<< endl;
    //     // cout << "Vel4\t" << vels.col(4)(0) << "\t" << vels.col(4)(1) << "\t" << vels.col(4)(2) << endl;
    //     // cout << "Acc4\t" << accs.col(4)(0) << "\t" << accs.col(4)(1) << "\t" << accs.col(4)(2) << endl;
    //     // cout << "Vel5\t" << vels.col(5)(0) << "\t" << vels.col(5)(1) << "\t" << vels.col(5)(2) << endl;
    //     // cout << "Acc5\t" << accs.col(5)(0) << "\t" << accs.col(5)(1) << "\t" << accs.col(5)(2) << endl;
    //     // cout << "Vel6\t" << vels.col(6)(0) << "\t" << vels.col(6)(1) << "\t" << vels.col(6)(2) << endl;
    //     // cout << "Acc6\t" << accs.col(6)(0) << "\t" << accs.col(6)(1) << "\t" << accs.col(6)(2)<< endl;
    //     // cout << "Vel7\t" << vels.col(7)(0) << "\t" << vels.col(7)(1) << "\t" << vels.col(7)(2) << endl;
    //     // cout << "Acc7\t" << accs.col(7)(0) << "\t" << accs.col(7)(1) << "\t" << accs.col(7)(2) << endl;
    //     // cout << "Vel8\t" << vels.col(8)(0) << "\t" << vels.col(8)(1) << "\t" << vels.col(8)(2) << endl;
    //     // cout << "Acc8\t" << accs.col(8)(0) << "\t" << accs.col(8)(1) << "\t" << accs.col(8)(2) << endl;
    //     // cout << "Vel9\t" << vels.col(9)(0) << "\t" << vels.col(9)(1) << "\t" << vels.col(9)(2) << endl;
    //     // cout << "Acc9\t" << accs.col(9)(0) << "\t" << accs.col(9)(1) << "\t" << accs.col(9)(2)<< endl;
    //     // cout << "Vel10\t" << vels.col(10)(0) << "\t" << vels.col(10)(1) << "\t" << vels.col(10)(2) << endl;
    //     // cout << "Acc10\t" << accs.col(10)(0) << "\t" << accs.col(10)(1) << "\t" << accs.col(10)(2) << endl;
    // }

    // inline void traj_high_speed(int &pieceN, 
    //                             Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                             Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    // {
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 7;
    //     int velCols = 5;
    //     int accCols = 0;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = posOffset(0) + 2.5;
    //     pts.col(0)(1) = posOffset(1) - 2.4;
    //     pts.col(0)(2) = posOffset(2) + 1.0 + init_z;
    //     vel = 3;
    //     vec << 1.0, 0.0, -0.3;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;

    //     pts.col(1)(0) = posOffset(0) + 5.0;
    //     pts.col(1)(1) = posOffset(1) + 0.0;
    //     pts.col(1)(2) = posOffset(2) + 0.4 + init_z;
    //     vel = 3.2;
    //     vec << 0.0, 1.0, -0.3;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 1;


    //     pts.col(2)(0) = posOffset(0) + 3;
    //     pts.col(2)(1) = posOffset(1) + 2.4;
    //     pts.col(2)(2) = posOffset(2) + 0.0 + init_z;
    //     vel = 3.2;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 2;

    //     pts.col(3)(0) = posOffset(0) + 0.0;
    //     pts.col(3)(1) = posOffset(1) + 1.2;
    //     pts.col(3)(2) = posOffset(2) + 0.75 + init_z;
    //     // vel = 5;
    //     // vec << -1.3, -1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(2) = vel * vec;
    //     // velNums(2) = 3;

    //     pts.col(4)(0) = posOffset(0) -3.0;
    //     pts.col(4)(1) = posOffset(1) -1.2;
    //     pts.col(4)(2) = posOffset(2) + 0.0 + init_z;
    //     vel = 3;
    //     vec << -3.5, -1.8, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 4;

    //     pts.col(5)(0) = posOffset(0) -1.8;
    //     pts.col(5)(1) = posOffset(1) -2.0;
    //     pts.col(5)(2) = posOffset(2) + 1.2 + init_z;
    //     vel = 2.7;
    //     vec << 3.5, 1.6, 0.0;
    //     vec.normalize();
    //     vels.col(4) = vel * vec;
    //     velNums(4) = 5;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    // }

    // inline void traj_high_speed(int &pieceN,
    //                             Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels, Eigen::Matrix3Xd &accs,
    //                             Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    // {

    //     // # PosOffset:                  [-1.0, 0.0, 0.0]      # high speed

    //     // # InitLoadPos:                [-1.5, 2.5, 0.6]     # high speed

    //     // # EndLoadPos:                [-6.0, 2.5, 0.6]     # high speed

    //     // # WeightT:                    1000.0             # high speed

    //     // # WeightE:                    0.3               #   high speed

    //     // # ChiVec:                     [1.0, 1.0, 10000.0, 100.0, 1.0, 1.0, 10000.0]   # pos, vel, collision, omg, tiltAngle, thrust, uniformforce   high speed

    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 8;
    //     int velCols = 6;
    //     int accCols = 0;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = posOffset(0) + 3.0;
    //     pts.col(0)(1) = posOffset(1) -2.4;
    //     pts.col(0)(2) = posOffset(2) + 1.0 + init_z;
    //     vel = 3;
    //     vec << 1.0, 0.0, -0.3;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;

    //     // pts.col(1)(0) = posOffset(0) + 5.0;
    //     // pts.col(1)(1) = posOffset(1) + 0.0;
    //     // pts.col(1)(2) = posOffset(2) + 0.4 + init_z;
    //     // vel = 3.2;
    //     // vec << 0.0, 1.0, -0.3;
    //     // vec.normalize();
    //     // vels.col(1) = vel * vec;
    //     // velNums(1) = 1;

    //     pts.col(1)(0) = posOffset(0) + 3.2;
    //     pts.col(1)(1) = posOffset(1) + 1.2;
    //     pts.col(1)(2) = posOffset(2) + 0.2 + init_z;
    //     vel = 3.2;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 1;

    //     pts.col(2)(0) = posOffset(0) + 3.2;
    //     pts.col(2)(1) = posOffset(1) - 1.2;
    //     pts.col(2)(2) = posOffset(2) + 0.8 + init_z;
    //     vel = 3.2;
    //     vec << 1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 2;

    //     // pts.col(3)(0) = posOffset(0) + 5.0;
    //     // pts.col(3)(1) = posOffset(1) + 0.0;
    //     // pts.col(3)(2) = posOffset(2) + 0.4 + init_z;
    //     // vel = 3.2;
    //     // vec << 0.0, 1.0, -0.3;
    //     // vec.normalize();
    //     // vels.col(3) = vel * vec;
    //     // velNums(3) = 3;

    //     pts.col(3)(0) = posOffset(0) + 3.0;
    //     pts.col(3)(1) = posOffset(1) + 2.4;
    //     pts.col(3)(2) = posOffset(2) + 0.0 + init_z;
    //     vel = 3.2;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 3;

    //     pts.col(4)(0) = posOffset(0) + 0.0;
    //     pts.col(4)(1) = posOffset(1) + 1.2;
    //     pts.col(4)(2) = posOffset(2) + 0.75 + init_z;
    //     // vel = 5;
    //     // vec << -1.3, -1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(2) = vel * vec;
    //     // velNums(2) = 3;

    //     pts.col(5)(0) = posOffset(0) -3.0;
    //     pts.col(5)(1) = posOffset(1) -1.2;
    //     pts.col(5)(2) = posOffset(2) + 0.0 + init_z;
    //     vel = 3;
    //     vec << -3.5, -1.8, 0.0;
    //     vec.normalize();
    //     vels.col(4) = vel * vec;
    //     velNums(4) = 5;

    //     pts.col(6)(0) = posOffset(0) - 1.8;
    //     pts.col(6)(1) = posOffset(1) - 2.0;
    //     pts.col(6)(2) = posOffset(2) + 1.2 + init_z;
    //     vel = 2.7;
    //     vec << 3.5, 1.6, 0.0;
    //     vec.normalize();
    //     vels.col(5) = vel * vec;
    //     velNums(5) = 6;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    // }

     inline void traj_high_speed(int &pieceN,
                                Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels, Eigen::Matrix3Xd &accs,
                                Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, Eigen::Vector3d &posOffset, const double &init_z)
    {

        // # PosOffset:                  [-1.0, 0.0, 0.0]      # high speed

        // # InitLoadPos:                [-1.5, 2.5, 0.6]     # high speed

        // # EndLoadPos:                [-6.0, 2.5, 0.6]     # high speed

        // # WeightT:                    1000.0             # high speed

        // # WeightE:                    0.3               #   high speed

        // # ChiVec:                     [1.0, 1.0, 10000.0, 100.0, 1.0, 1.0, 10000.0]   # pos, vel, collision, omg, tiltAngle, thrust, uniformforce   high speed

        Eigen::Vector3d vec;
        double gra_ = 9.8;
        double len = 1.6;
        double v = 3.6;
        pieceN = 8;
        int velCols = 6;
        int accCols = 0;
        double theta = M_PI / 4.0;
        pts.resize(3, pieceN - 1);
        vels.resize(3, velCols);
        accs.resize(3, accCols);
        velNums.resize(velCols);
        accNums.resize(accCols);
        pts.col(0)(0) = posOffset(0) + 3.0;
        pts.col(0)(1) = posOffset(1) -2.4;
        pts.col(0)(2) = posOffset(2) + 0.8 + init_z;
        vel = 2.5;
        vec << 1.0, 0.0, -0.3;
        vec.normalize();
        vels.col(0) = vel * vec;
        velNums(0) = 0;

        // pts.col(1)(0) = posOffset(0) + 5.0;
        // pts.col(1)(1) = posOffset(1) + 0.0;
        // pts.col(1)(2) = posOffset(2) + 0.4 + init_z;
        // vel = 3.2;
        // vec << 0.0, 1.0, -0.3;
        // vec.normalize();
        // vels.col(1) = vel * vec;
        // velNums(1) = 1;

        pts.col(1)(0) = posOffset(0) + 3.2;
        pts.col(1)(1) = posOffset(1) + 1.2;
        pts.col(1)(2) = posOffset(2) + 0.15 + init_z;
        vel = 2.7;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(1) = vel * vec;
        velNums(1) = 1;

        pts.col(2)(0) = posOffset(0) + 3.2;
        pts.col(2)(1) = posOffset(1) - 1.2;
        pts.col(2)(2) = posOffset(2) + 0.65 + init_z;
        vel = 2.7;
        vec << 1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(2) = vel * vec;
        velNums(2) = 2;

        // pts.col(3)(0) = posOffset(0) + 5.0;
        // pts.col(3)(1) = posOffset(1) + 0.0;
        // pts.col(3)(2) = posOffset(2) + 0.4 + init_z;
        // vel = 3.2;
        // vec << 0.0, 1.0, -0.3;
        // vec.normalize();
        // vels.col(3) = vel * vec;
        // velNums(3) = 3;

        pts.col(3)(0) = posOffset(0) + 3.0;
        pts.col(3)(1) = posOffset(1) + 2.4;
        pts.col(3)(2) = posOffset(2) + 0.0 + init_z;
        vel = 2.7;
        vec << -1.0, 0.0, 0.0;
        vec.normalize();
        vels.col(3) = vel * vec;
        velNums(3) = 3;

        pts.col(4)(0) = posOffset(0) + 0.0;
        pts.col(4)(1) = posOffset(1) + 1.1;
        pts.col(4)(2) = posOffset(2) + 0.75 + init_z;
        // vel = 5;
        // vec << -1.3, -1.0, 0.0;
        // vec.normalize();
        // vels.col(2) = vel * vec;
        // velNums(2) = 3;

        pts.col(5)(0) = posOffset(0) - 2.8;
        pts.col(5)(1) = posOffset(1) -1.2;
        pts.col(5)(2) = posOffset(2) + 0.0 + init_z;
        vel = 2.5;
        vec << -3.5, -1.8, 0.0;
        vec.normalize();
        vels.col(4) = vel * vec;
        velNums(4) = 5;

        pts.col(6)(0) = posOffset(0) - 1.6;
        pts.col(6)(1) = posOffset(1) - 2.0;
        pts.col(6)(2) = posOffset(2) + 1.2 + init_z;
        vel = 2.5;
        vec << 3.5, 1.6, 0.0;
        vec.normalize();
        vels.col(5) = vel * vec;
        velNums(5) = 6;

        cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
        cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
        cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
        cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
        cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
        cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
        cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
        cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
        cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    }

// inline void traj_high_speed(int &pieceN, 
//                                 Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
//                                 Eigen::VectorXi &velNums, Eigen::VectorXi &accNums, const double &init_z)
//     {
//         Eigen::Vector3d vec;
//         double gra_ = 9.8;
//         double len = 1.6;
//         double v = 3.6;
//         pieceN = 7;
//         int velCols = 5;
//         int accCols = 0;
//         double theta = M_PI / 4.0;
//         pts.resize(3, pieceN - 1);
//         vels.resize(3, velCols);
//         accs.resize(3, accCols);
//         velNums.resize(velCols);
//         accNums.resize(accCols);
//         pts.col(0)(0) = 2.5;
//         pts.col(0)(1) = -2.4;
//         pts.col(0)(2) = 1.0 + init_z;
//         vel = 2;
//         vec << 1.0, 0.0, -0.3;
//         vec.normalize();
//         vels.col(0) = vel * vec;
//         velNums(0) = 0;

//         pts.col(1)(0) = 5.0;
//         pts.col(1)(1) = 0.0;
//         pts.col(1)(2) = 0.4 + init_z;
//         vel = 2;
//         vec << 0.0, 1.0, -0.3;
//         vec.normalize();
//         vels.col(1) = vel * vec;
//         velNums(1) = 1;


//         pts.col(2)(0) = 3;
//         pts.col(2)(1) = 2.2;
//         pts.col(2)(2) = 0.0 + init_z;
//         vel = 2;
//         vec << -1.0, 0.0, 0.0;
//         vec.normalize();
//         vels.col(2) = vel * vec;
//         velNums(2) = 2;

//         pts.col(3)(0) = 0.0;
//         pts.col(3)(1) = 1.1;
//         pts.col(3)(2) = 0.75 + init_z;
//         // vel = 5;
//         // vec << -1.3, -1.0, 0.0;
//         // vec.normalize();
//         // vels.col(2) = vel * vec;
//         // velNums(2) = 3;

//         pts.col(4)(0) = -3.0;
//         pts.col(4)(1) = -1.2;
//         pts.col(4)(2) = 0.0 + init_z;
//         vel = 2;
//         vec << -3.5, -1.8, 0.0;
//         vec.normalize();
//         vels.col(3) = vel * vec;
//         velNums(3) = 4;

//         pts.col(5)(0) = -1.8;
//         pts.col(5)(1) = -2.0;
//         pts.col(5)(2) = 1.2 + init_z;
//         vel = 2;
//         vec << 3.5, 1.6, 0.0;
//         vec.normalize();
//         vels.col(4) = vel * vec;
//         velNums(4) = 5;

//         cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
//         cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
//         cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
//         cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
//         cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
//         cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
//         cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
//         cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
//         cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
//     }

    //  inline void traj_high_speed(int &pieceN, 
    //                             Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                             Eigen::VectorXi &velNums, Eigen::VectorXi &accNums)
    // {
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 7;
    //     int velCols = 4;
    //     int accCols = 0;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = 2;
    //     pts.col(0)(1) = -2;
    //     pts.col(0)(2) = 1.0;
    //     vel = 7;
    //     vec << 1.0, 0.0, -0.3;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;

    //     pts.col(1)(0) = 4.2;
    //     pts.col(1)(1) = 0.0;
    //     pts.col(1)(2) = 1.0;

    //     pts.col(2)(0) = 3;
    //     pts.col(2)(1) = 2;
    //     pts.col(2)(2) = 0.0;
    //     vel = 5;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 2;

    //     pts.col(3)(0) = 0.0;
    //     pts.col(3)(1) = 1.25;
    //     pts.col(3)(2) = 0.5;
    //     // vel = 5;
    //     // vec << -1.3, -1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(2) = vel * vec;
    //     // velNums(2) = 3;

    //     pts.col(4)(0) = -3.5;
    //     pts.col(4)(1) = -1.2;
    //     pts.col(4)(2) = 0.0;
    //     vel = 4;
    //     vec << -3.5, -1.2, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 4;

    //     pts.col(5)(0) = -3.5;
    //     pts.col(5)(1) = -1.6;
    //     pts.col(5)(2) = 2.0;
    //     vel = 4;
    //     vec << 3.5, 1.6, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 5;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    // }


    // inline void traj_high_speed(int &pieceN, 
    //                             Eigen::Matrix3Xd &pts, Eigen::Matrix3Xd &vels,Eigen::Matrix3Xd &accs,
    //                             Eigen::VectorXi &velNums, Eigen::VectorXi &accNums)
    // {
    //     Eigen::Vector3d vec;
    //     double gra_ = 9.8;
    //     double len = 1.6;
    //     double v = 3.6;
    //     pieceN = 6;
    //     int velCols = 5;
    //     int accCols = 0;
    //     double theta = M_PI / 4.0;
    //     pts.resize(3, pieceN - 1);
    //     vels.resize(3, velCols);
    //     accs.resize(3, accCols);
    //     velNums.resize(velCols);
    //     accNums.resize(accCols);
    //     pts.col(0)(0) = 2;
    //     pts.col(0)(1) = -2;
    //     pts.col(0)(2) = 1.0;
    //     vel = 7;
    //     vec << 1.0, 0.0, -0.3;
    //     vec.normalize();
    //     vels.col(0) = vel * vec;
    //     velNums(0) = 0;
    //     // acc = vel * vel / 5.0;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(0) = acc * vec;
    //     // accNums(0) = 0;

    //     pts.col(1)(0) = 5.0;
    //     pts.col(1)(1) = 0.0;
    //     pts.col(1)(2) = 0.5;
    //     // vel = 5;
    //     // vec << 0.0, 1.0, -0.15;
    //     // vec.normalize();
    //     // vels.col(1) = vel * vec;
    //     // velNums(1) = 1;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(1) = acc * vec;
    //     // accNums(1) = 1;

    //     pts.col(2)(0) = 3;
    //     pts.col(2)(1) = 2;
    //     pts.col(2)(2) = 0.0;
    //     vel = 3;
    //     vec << -1.0, 0.0, 0.0;
    //     vec.normalize();
    //     vels.col(1) = vel * vec;
    //     velNums(1) = 1;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(2) = acc * vec;
    //     // accNums(2) = 2;

    //     pts.col(3)(0) = -3.5;
    //     pts.col(3)(1) = -1.2;
    //     pts.col(3)(2) = 0.0;
    //     vel = 4;
    //     vec << -3.5, -1.2, 0.0;
    //     vec.normalize();
    //     vels.col(2) = vel * vec;
    //     velNums(2) = 2;
    //     // acc = vel * vel / len;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(3) = acc * vec;
    //     // accNums(3) = 3;

    //     pts.col(4)(0) = -3.5;
    //     pts.col(4)(1) = -1.2;
    //     pts.col(4)(2) = 2.0;
    //     vel = 4;
    //     vec << 3.5, 1.2, 0.0;
    //     vec.normalize();
    //     vels.col(3) = vel * vec;
    //     velNums(3) = 3;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(4) = acc * vec;
    //     // accNums(4) = 4;

    //     // pts.col(5)(0) = 2 * len;
    //     // pts.col(5)(1) = 0.0;
    //     // pts.col(5)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, 1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(5) = vel * vec;
    //     // velNums(5) = 5;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(5) = acc * vec;
    //     // accNums(5) = 5;

    //     // pts.col(6)(0) = len;
    //     // pts.col(6)(1) = len;
    //     // pts.col(6)(2) = 0.0;
    //     // vel = v;
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(6) = vel * vec;
    //     // velNums(6) = 6;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(6) = acc * vec;
    //     // accNums(6) = 6;

    //     // pts.col(7)(0) = 0.0;
    //     // pts.col(7)(1) = 0.0;
    //     // pts.col(7)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, -1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(7) = vel * vec;
    //     // velNums(7) = 7;
    //     // acc = vel * vel / len;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(7) = acc * vec;
    //     // accNums(7) = 7;

    //     // pts.col(8)(0) = len;
    //     // pts.col(8)(1) = -len;
    //     // pts.col(8)(2) = 0.0;
    //     // vel = v;
    //     // vec << 1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(8) = vel * vec;
    //     // velNums(8) = 8;
    //     // acc = vel * vel / len;
    //     // vec << 0.0, 1.0, 0.0;
    //     // cout << vec<< endl;
    //     // vec.normalize();
    //     // accs.col(8) = acc * vec;
    //     // accNums(8) = 8;

    //     // pts.col(9)(0) = 2 * len;
    //     // pts.col(9)(1) = 0.0;
    //     // pts.col(9)(2) = 0.0;
    //     // vel = v;
    //     // vec << 0.0, 1.0, 0.0;
    //     // vec.normalize();
    //     // vels.col(9) = vel * vec;
    //     // velNums(9) = 9;
    //     // acc = vel * vel / len; // an
    //     // vec << -1.0, 0.0, 0.0;
    //     // vec.normalize();
    //     // accs.col(9) = acc * vec;
    //     // accNums(9) = 9;

    //     // pts.col(10)(0) = len + len * cos(theta);
    //     // pts.col(10)(1) = len * sin(theta);
    //     // pts.col(10)(2) = 0.0;
    //     // vel = v;
    //     // vec << -sin(theta), cos(theta), 0.0;
    //     // vec.normalize();
    //     // vels.col(10) = vel * vec;
    //     // velNums(10) = 10;
    //     // acc = vel * vel / len;
    //     // vec << -cos(theta), -sin(theta), 0.0;
    //     // vec.normalize();
    //     // accs.col(10) = acc * vec;
    //     // accNums(10) = 10;

    //     cout << "Pts0\t" << pts.col(0)(0) << "\t" << pts.col(0)(1) << "\t" << pts.col(0)(2) << endl;
    //     cout << "Pts1\t" << pts.col(1)(0) << "\t" << pts.col(1)(1) << "\t" << pts.col(1)(2) << endl;
    //     cout << "Pts2\t" << pts.col(2)(0) << "\t" << pts.col(2)(1) << "\t" << pts.col(2)(2) << endl;
    //     cout << "Pts3\t" << pts.col(3)(0) << "\t" << pts.col(3)(1) << "\t" << pts.col(3)(2) << endl;
    //     cout << "Pts4\t" << pts.col(4)(0) << "\t" << pts.col(4)(1) << "\t" << pts.col(4)(2) << endl;
    //     // cout << "Pts5\t" << pts.col(5)(0) << "\t" << pts.col(5)(1) << "\t" << pts.col(5)(2) << endl;
    //     // cout << "Pts6\t" << pts.col(6)(0) << "\t" << pts.col(6)(1) << "\t" << pts.col(6)(2) << endl;
    //     // cout << "Pts7\t" << pts.col(7)(0) << "\t" << pts.col(7)(1) << "\t" << pts.col(7)(2) << endl;
    //     // cout << "Pts8\t" << pts.col(8)(0) << "\t" << pts.col(8)(1) << "\t" << pts.col(8)(2) << endl;
    //     // cout << "Pts9\t" << pts.col(9)(0) << "\t" << pts.col(9)(1) << "\t" << pts.col(9)(2) << endl;
    //     // cout << "Pts10\t" << pts.col(10)(0) << "\t" << pts.col(10)(1) << "\t" << pts.col(10)(2) << endl;

    //     cout << "Vel0\t" << vels.col(0)(0) << "\t" << vels.col(0)(1) << "\t" << vels.col(0)(2) << endl;
    //     // cout << "Acc0\t" << accs.col(0)(0) << "\t" << accs.col(0)(1) << "\t" << accs.col(0)(2)<< endl;
    //     cout << "Vel1\t" << vels.col(1)(0) << "\t" << vels.col(1)(1) << "\t" << vels.col(1)(2) << endl;
    //     // cout << "Acc1\t" << accs.col(1)(0) << "\t" << accs.col(1)(1) << "\t" << accs.col(1)(2) << endl;
    //     cout << "Vel2\t" << vels.col(2)(0) << "\t" << vels.col(2)(1) << "\t" << vels.col(2)(2) << endl;
    //     // cout << "Acc2\t" << accs.col(2)(0) << "\t" << accs.col(2)(1) << "\t" << accs.col(2)(2) << endl;
    //     cout << "Vel3\t" << vels.col(3)(0) << "\t" << vels.col(3)(1) << "\t" << vels.col(3)(2) << endl;
    //     // cout << "Acc3\t" << accs.col(3)(0) << "\t" << accs.col(3)(1) << "\t" << accs.col(3)(2)<< endl;
    //     cout << "Vel4\t" << vels.col(4)(0) << "\t" << vels.col(4)(1) << "\t" << vels.col(4)(2) << endl;
    //     // cout << "Acc4\t" << accs.col(4)(0) << "\t" << accs.col(4)(1) << "\t" << accs.col(4)(2) << endl;
    //     // cout << "Vel5\t" << vels.col(5)(0) << "\t" << vels.col(5)(1) << "\t" << vels.col(5)(2) << endl;
    //     // cout << "Acc5\t" << accs.col(5)(0) << "\t" << accs.col(5)(1) << "\t" << accs.col(5)(2) << endl;
    //     // cout << "Vel6\t" << vels.col(6)(0) << "\t" << vels.col(6)(1) << "\t" << vels.col(6)(2) << endl;
    //     // cout << "Acc6\t" << accs.col(6)(0) << "\t" << accs.col(6)(1) << "\t" << accs.col(6)(2)<< endl;
    //     // cout << "Vel7\t" << vels.col(7)(0) << "\t" << vels.col(7)(1) << "\t" << vels.col(7)(2) << endl;
    //     // cout << "Acc7\t" << accs.col(7)(0) << "\t" << accs.col(7)(1) << "\t" << accs.col(7)(2) << endl;
    //     // cout << "Vel8\t" << vels.col(8)(0) << "\t" << vels.col(8)(1) << "\t" << vels.col(8)(2) << endl;
    //     // cout << "Acc8\t" << accs.col(8)(0) << "\t" << accs.col(8)(1) << "\t" << accs.col(8)(2) << endl;
    //     // cout << "Vel9\t" << vels.col(9)(0) << "\t" << vels.col(9)(1) << "\t" << vels.col(9)(2) << endl;
    //     // cout << "Acc9\t" << accs.col(9)(0) << "\t" << accs.col(9)(1) << "\t" << accs.col(9)(2)<< endl;
    //     // cout << "Vel10\t" << vels.col(10)(0) << "\t" << vels.col(10)(1) << "\t" << vels.col(10)(2) << endl;
    //     // cout << "Acc10\t" << accs.col(10)(0) << "\t" << accs.col(10)(1) << "\t" << accs.col(10)(2) << endl;
    // }

}