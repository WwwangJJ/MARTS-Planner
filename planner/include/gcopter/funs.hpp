#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
using namespace std;

namespace gcopter
{
    double g1 = 0, g2 = 0, g3 = 0, g4 = 0;
    Eigen::VectorXd test0(6), test1(6);
    Eigen::Vector3d pqpt_, pqpp_, pdqpt, pdqpp, pdqpdt, pdqpdp, pd2qpt, pd2qpp, pd2qpdt, pd2qpdp, pd2qpd2t, pd2qpd2p, pd3qpt, pd3qpp, pd3qpdt, pd3qpdp, pd3qpd2t, pd3qpd2p, pd3qpd3t, pd3qpd3p;  
    Eigen::Vector3d pd4qpt, pd4qpp, pd4qpdt, pd4qpdp, pd4qpd2t, pd4qpd2p, pd4qpd3t, pd4qpd3p, pd4qpd4t, pd4qpd4p; 
    inline bool smoothedL1(const double &x,
                           const double &mu,
                           double &f,
                           double &df)
    {

        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }
    inline Eigen::Vector2d getcs(const double &ang)
    {
        Eigen::Vector2d cs;
        cs(0) = cos(ang);
        cs(1) = sin(ang);
        return cs;
    }

    inline void setg(const Eigen::Vector4d &g)
    {
        g1 = g(0);
        g2 = g(1);
        g3 = g(2);
        g4 = g(3);
        return;
    }

    inline void set1g()
    {
        g1 += 1.0;
        g2 += 1.0;
        g3 += 1.0;
        g4 += 1.0;
        return;
    }

    // inline Eigen::Vector4d getG(const Eigen::Vector2d &cst,
    //                             const Eigen::Vector2d &csp)
    // {
    //     Eigen::Vector4d g;
    //     g(0) = cst(1) * csp(0);
    //     g(1) = cst(1) * csp(1);
    //     g(2) = cst(0) * csp(0);
    //     g(3) = cst(0) * csp(1);
    //     return g;
    // }

    inline Eigen::Vector3d getQ(const Eigen::Vector2d &cst,
                                const Eigen::Vector2d &csp,
                                Eigen::Vector4d &g)
    {
        Eigen::Vector3d q;
        g(0) = cst(1) * csp(0);
        g(1) = cst(1) * csp(1);
        g(2) = cst(0) * csp(0);
        g(3) = cst(0) * csp(1);
        q(0) = g(0);
        q(1) = g(1);
        q(2) = cst(0);
        return q;
    }

    inline Eigen::Vector3d getdQ(const Eigen::Vector2d &dAng, const Eigen::Vector2d &cst, const Eigen::Vector4d &gs)
    {
        Eigen::Vector3d dq;
        Eigen::Matrix<double, 2, 1> base;
        Eigen::Matrix<double, 3, 2> cof;
        g1 = gs(0);
        g2 = gs(1);
        g3 = gs(2);
        g4 = gs(3);

        base << dAng(0), dAng(1);
        cof.row(0) << g3, -g2;
        cof.row(1) << g4, g1;
        cof.row(2) << -cst(1), 0.0;
        dq = (cof * base).col(0);
        return dq;
    }

    inline Eigen::Vector3d getd2Q(const Eigen::Vector2d &dAng,
                                  const Eigen::Vector2d &d2Ang,
                                  const Eigen::Vector2d &cst)
    {
        Eigen::Vector3d d2q;
        Eigen::Matrix<double, 5, 1> base;
        Eigen::Matrix<double, 3, 5> cof;
        // cout << "row0\t"<< cof.row(0)(0) << "\t"<< cof.row(0)(1) << "\t"<< cof.row(0)(2) << "\t"<< cof.row(0)(3) << "\t"<< cof.row(0)(4) << "\t"<< endl;
        // cout << "row1\t"<< cof.row(1)(0) << "\t"<< cof.row(1)(1) << "\t"<< cof.row(1)(2) << "\t"<< cof.row(1)(3) << "\t"<< cof.row(1)(4) << "\t"<< endl;
        // cout << "row2\t"<< cof.row(2)(0) << "\t"<< cof.row(2)(1) << "\t"<< cof.row(2)(2) << "\t"<< cof.row(2)(3) << "\t"<< cof.row(2)(4) << "\t"<< endl;
        base << dAng(0) * dAng(0), dAng(0) * dAng(1), d2Ang(0), dAng(1) * dAng(1), d2Ang(1);
        cof.row(0) << -g1, -2 * g4, g3, -g1, -g2;
        cof.row(1) << -g2, 2 * g3, g4, -g2, g1;
        cof.row(2) << -cst(0), 0.0, -cst(1), 0.0, 0.0;
        d2q = (cof * base).col(0);
        // cout << "row1_\t"<< cof.row(1)(0) << "\t"<< cof.row(1)(1) << "\t"<< cof.row(1)(2) << "\t"<< cof.row(1)(3) << "\t"<< cof.row(1)(4) << "\t"<< endl;
        // cout << "row2_\t"<< cof.row(2)(0) << "\t"<< cof.row(2)(1) << "\t"<< cof.row(2)(2) << "\t"<< cof.row(2)(3) << "\t"<< cof.row(2)(4) << "\t"<< endl;
        // cout << "base\t"<< base(0) << "\t"<< base(1) << "\t"<< base(2) << "\t"<< base(3) << "\t"<< base(4) << "\t"<< endl;
        // cout << "d2q\t"<< d2q(0) << "\t"<< d2q(1) << "\t"<< d2q(2) << "\t"<< endl;
        return d2q;
    }

    inline Eigen::Vector3d getd3Q(const Eigen::Vector2d &dAng,
                                  const Eigen::Vector2d &d2Ang,
                                  const Eigen::Vector2d &d3Ang,
                                  const Eigen::Vector2d &cst)
    {
        Eigen::Vector3d d3q;
        Eigen::Matrix<double, 10, 1> base;
        Eigen::Matrix<double, 3, 10> cof;
        base << pow(dAng(0), 3), pow(dAng(0), 2) * dAng(1), dAng(0) * d2Ang(0), dAng(0) * pow(dAng(1), 2), d2Ang(0) * dAng(1),
            dAng(0) * d2Ang(1), d3Ang(0), pow(dAng(1), 3), dAng(1) * d2Ang(1), d3Ang(1);
        cof.row(0) << -g3, 3 * g2, -3 * g1, -3 * g3, -3 * g4, -3 * g4, g3, g2, -3 * g1, -g2;
        cof.row(1) << -g4, -3 * g1, -3 * g2, -3 * g4, 3 * g3, 3 * g3, g4, -g1, -3 * g2, g1;
        ;
        cof.row(2) << cst(1), 0.0, -3 * cst(0), 0.0, 0.0, 0.0, -cst(1), 0.0, 0.0, 0.0;
        d3q = (cof * base).col(0);
        return d3q;
    }

    inline Eigen::Vector3d getd4Q(const Eigen::Vector2d &dAng,
                                  const Eigen::Vector2d &d2Ang,
                                  const Eigen::Vector2d &d3Ang,
                                  const Eigen::Vector2d &d4Ang,
                                  const Eigen::Vector2d &cst,
                                  const Eigen::Vector2d &csp)
    {
        Eigen::Vector3d d4q;
        d4q(0) = 6 * (-cst(1) * pow(dAng(0), 2) + cst(0) * d2Ang(0)) * (-csp(0) * pow(dAng(1), 2) - csp(1) * d2Ang(1)) -
                 4 * csp(1) * dAng(1) * (-cst(0) * pow(dAng(0), 3) - 3 * cst(1) * dAng(0) * d2Ang(0) + cst(0) * d3Ang(0)) +
                 4 * cst(0) * dAng(0) * (csp(1) * pow(dAng(1), 3) - 3 * csp(0) * dAng(1) * d2Ang(1) - csp(1) * d3Ang(1)) +
                 csp(0) * (cst(1) * pow(dAng(0), 4) - 6 * cst(0) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(1) * pow(d2Ang(0), 2) - 4 * cst(1) * dAng(0) * d3Ang(0) + cst(0) * d4Ang(0)) +
                 cst(1) * (csp(0) * pow(dAng(1), 4) + 6 * csp(1) * pow(dAng(1), 2) * d2Ang(1) - 3 * csp(0) * pow(d2Ang(1), 2) - 4 * csp(0) * dAng(1) * d3Ang(1) - csp(1) * d4Ang(1));
        d4q(1) = 6 * (-cst(1) * pow(dAng(0), 2) + cst(0) * d2Ang(0)) * (-csp(1) * pow(dAng(1), 2) + csp(0) * d2Ang(1)) +
                 4 * csp(0) * dAng(1) * (-cst(0) * pow(dAng(0), 3) - 3 * cst(1) * dAng(0) * d2Ang(0) + cst(0) * d3Ang(0)) +
                 4 * cst(0) * dAng(0) * (-csp(0) * pow(dAng(1), 3) - 3 * csp(1) * dAng(1) * d2Ang(1) + csp(0) * d3Ang(1)) +
                 csp(1) * (cst(1) * pow(dAng(0), 4) - 6 * cst(0) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(1) * pow(d2Ang(0), 2) - 4 * cst(1) * dAng(0) * d3Ang(0) + cst(0) * d4Ang(0)) +
                 cst(1) * (csp(1) * pow(dAng(1), 4) - 6 * csp(0) * pow(dAng(1), 2) * d2Ang(1) - 3 * csp(1) * pow(d2Ang(1), 2) - 4 * csp(1) * dAng(1) * d3Ang(1) + csp(0) * d4Ang(1));
        d4q(2) = cst(0) * pow(dAng(0), 4) + 6 * cst(1) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(0) * pow(d2Ang(0), 2) - 4 * cst(0) * dAng(0) * d3Ang(0) - cst(1) * d4Ang(0);

        return d4q;
    }

    inline void getd5Qs(const Eigen::Vector2d &dAng,
                                  const Eigen::Vector2d &d2Ang,
                                  const Eigen::Vector2d &d3Ang,
                                  const Eigen::Vector2d &d4Ang,
                                  const Eigen::Vector2d &d5Ang,
                                  const Eigen::Vector2d &cst,
                                  const Eigen::Vector2d &csp,
                                  const Eigen::Vector4d &g,
                                  Eigen::VectorXd &d2qb,
                                  Eigen::VectorXd &d3qb,
                                  Eigen::VectorXd &d4qb,
                                  Eigen::VectorXd &d5qb,
                                  Eigen::Vector3d &dq,
                                  Eigen::Vector3d &d2q,
                                  Eigen::Vector3d &d3q,
                                  Eigen::Vector3d &d4q,
                                  Eigen::Vector3d &d5q)
    {   
        dq(0) =  g(2) * dAng(0) - g(1) * dAng(1);
        dq(1) =  g(3) * dAng(0) + g(0) * dAng(1);
        dq(2) =  -cst(1) * dAng(0);

        d2qb(0) = d2Ang(1);
        d2qb(1) = dAng(0) * dAng(1);
        d2qb(2) = pow(dAng(0), 2) +  pow(dAng(1), 2);
        d2qb(3) = d2Ang(0);

        d2q(0) =  -g(1) * d2qb(0) - 2 * g(3) * d2qb(1) - g(0) * d2qb(2) + g(2) * d2qb(3);
        d2q(1) =  -g(1) * d2qb(2) + g(3) * d2qb(3) + g(0) * d2qb(0) + 2 * g(2) * d2qb(1);
        d2q(2) =  -cst(1) * d2Ang(0) - cst(0) * pow(dAng(0), 2);

        d3qb(0) = d3Ang(1) - pow(dAng(1), 3) - 3 * pow(dAng(0), 2) * dAng(1);
        d3qb(1) = 3 * dAng(0) * d2Ang(1) + 3 * d2Ang(0) * dAng(1);
        d3qb(2) = 3 * dAng(0) * d2Ang(0) + 3 * d2Ang(1) * dAng(1);
        d3qb(3) = -d3Ang(0) + pow(dAng(0), 3) + 3 * pow(dAng(1), 2) * dAng(0);
        d3qb(4) = pow(dAng(0), 3) - d3Ang(0);
        d3qb(5) = dAng(0) * d2Ang(0);

        d3q(0) =  -g(1) * d3qb(0) - g(3) * d3qb(1) - g(0) * d3qb(2) - g(2) * d3qb(3);
        d3q(1) =  -g(1) * d3qb(2) - g(3) * d3qb(3) + g(0) * d3qb(0) + g(2) * d3qb(1);
        d3q(2) =  cst(1) * d3qb(4) - 3 * cst(0) * d3qb(5);

        d4qb(0) = - d4Ang(1) + 6 * d2qb(2) * d2Ang(1) + 12 * d2qb(1) * d2Ang(0);
        d4qb(1) = -4 * dAng(0) * d3Ang(1) - 6 * d2Ang(0) * d2Ang(1) + 4 * dAng(0) * pow(dAng(1), 3) + 4 * d3qb(4) * dAng(1);
        d4qb(2) = -4 * dAng(1) * d3Ang(1) -  3 * pow(d2Ang(1), 2) + pow(dAng(1), 4) + 6 * pow(d2qb(1), 2) - 4 * dAng(0) * d3Ang(0) - 3 * pow(d2Ang(0), 2) + pow(dAng(0), 4);
        d4qb(3) = -12 * d2qb(1) * d2Ang(1) - 6 * d2Ang(0) *  pow(dAng(1), 2) +  d4Ang(0) - 6 * pow(dAng(0), 2) * d2Ang(0);
        d4qb(4) = 6 * pow(dAng(0), 2) * d2Ang(0) - d4Ang(0);
        d4qb(5) = -4 * dAng(0) * d3Ang(0) - 3 * pow(d2Ang(0), 2) + pow(dAng(0), 4);
          
        d4q(0) = g(1) * d4qb(0) + g(3) * d4qb(1) + g(0) * d4qb(2) + g(2) * d4qb(3);
        d4q(1) = g(1) * d4qb(2) + g(3) * d4qb(3) - g(0) * d4qb(0) - g(2) * d4qb(1);
        d4q(2) = cst(1) * d4qb(4) + cst(0) * d4qb(5);

        d5qb(0) = - d5Ang(1) + 10 * d2qb(2) * d3Ang(1) + 15 * dAng(1) * pow(d2Ang(1), 2) + 30 * dAng(0) * d2Ang(0) * d2Ang(1) - pow(dAng(1), 5) - 10 * pow(d2qb(1), 2) * dAng(1) - 5 * d4qb(5) * dAng(1);
        d5qb(1) = -5 * dAng(0) * d4Ang(1) - 10 * d2Ang(0) * d3Ang(1) + 10 * d3qb(3) * d2Ang(1) + 10 * d2Ang(0) * pow(dAng(1), 3)  + 5 * d4qb(4) * dAng(1);
        d5qb(2) = -5 * dAng(1) * d4Ang(1) - 10 * d3qb(0) * d2Ang(1) + 10 * d3qb(3) * d2Ang(0) - 5 * dAng(0) * d4Ang(0);
        d5qb(3) = -20 * d2qb(1) * d3Ang(1) - 15 * dAng(0) * pow(d2Ang(1), 2) - 30 * d2Ang(0) * dAng(1) * d2Ang(1) + 5 * d2qb(1) * pow(dAng(1), 3) + 10 * d3qb(4) * pow(dAng(1), 2) + d5Ang(0) - 10 * pow(dAng(0), 2) * d3Ang(0) - 15 * dAng(0) * pow(d2Ang(0), 2) + pow(dAng(0), 5);


        d5q(0) = g(1) * d5qb(0) + g(3) * d5qb(1) + g(0) * d5qb(2) + g(2) * d5qb(3);
        d5q(1) = g(1) * d5qb(2) + g(3) * d5qb(3) - g(0) * d5qb(0) - g(2) * d5qb(1);
        d5q(2) = cst(1) * (- d5Ang(0) + 10 * pow(dAng(0), 2) * d3Ang(0) + 15 * dAng(0) * pow(d2Ang(0), 2) - pow(dAng(0), 5)) + cst(0) * (-5 * dAng(0) * d4Ang(0) + 10 * d2Ang(0) * d3qb(4));
        return;
    }

    inline void getPartialqbyCoeff(const Eigen::Vector3d &gradq,
                                   const Eigen::Vector2d &cst,
                                   const Eigen::VectorXd &beta0,
                                   Eigen::MatrixX2d &partGradbyCoeff)
    {
        Eigen::Vector3d pqpt, pqpp;
        pqpt(0) = g3;
        pqpt(1) = g4;
        pqpt(2) = -cst(1);
        pqpp(0) = -g2;
        pqpp(1) = g1;
        pqpp(2) = 0.0;
        partGradbyCoeff.col(0) = gradq.dot(pqpt) * beta0;
        partGradbyCoeff.col(1) = gradq.dot(pqpp) * beta0;
        // cout << "gradqt\t" <<partGradbyCoeff.col(0)(0) << "\t" << partGradbyCoeff.col(0)(1) << "\t" << partGradbyCoeff.col(0)(2) << "\t" << partGradbyCoeff.col(0)(3) << "\t" <<partGradbyCoeff.col(0)(4) << "\t" << partGradbyCoeff.col(0)(5) << "\t"<< endl;
        // cout << "gradqp\t" <<partGradbyCoeff.col(1)(0) << "\t" << partGradbyCoeff.col(1)(1) << "\t" << partGradbyCoeff.col(1)(2) << "\t" << partGradbyCoeff.col(1)(3) << "\t" <<partGradbyCoeff.col(1)(4) << "\t" << partGradbyCoeff.col(1)(5) << "\t"<< endl;
    }

    inline void getPartialdqbyCoeff(const Eigen::Vector3d &graddq,
                                    const Eigen::Vector2d &dAng,
                                    const Eigen::Vector2d &cst,
                                    const Eigen::VectorXd &beta0,
                                    const Eigen::VectorXd &beta1,
                                    Eigen::MatrixX2d &partGradbyCoeff)
    {
        Eigen::Matrix<double, 3, 2> pdqptc, pdqppc;
        Eigen::Vector3d pdqpdt, pdqpdp;
        Eigen::Matrix<double, 2, 1> base1;
        base1 << dAng(0), dAng(1);
        pdqptc.row(0) << -g1, -g4;
        pdqptc.row(1) << -g2, g3;
        pdqptc.row(2) << -cst(0), 0.0;
        pdqpdt << g3, g4, -cst(1);

        pdqppc.row(0) << -g4, -g1;
        pdqppc.row(1) << g3, -g2;
        pdqppc.row(2) << 0.0, 0.0;
        pdqpdp << -g2, g1, 0.0;
        partGradbyCoeff.col(0) += graddq.dot(pdqptc * base1) * beta0 + graddq.dot(pdqpdt) * beta1;
        partGradbyCoeff.col(1) += graddq.dot(pdqppc * base1) * beta0 + graddq.dot(pdqpdp) * beta1;
        // test0 = graddq.dot(pdqptc * base1) * beta0 + graddq.dot(pdqpdt) * beta1;
        // test1 = graddq.dot(pdqppc * base1) * beta0 + graddq.dot(pdqpdp) * beta1;
        // cout << "dq\t" << test0(0) << "\t" << test0(1) << "\t"<< test0(2) << "\t"<< test0(3) << "\t"<< test0(4) << "\t"<< test0(5) << endl;
        // cout << "dq\t" << test1(0) << "\t" << test1(1) << "\t"<< test1(2) << "\t"<< test1(3) << "\t"<< test1(4) << "\t"<< test1(5) << endl;
        // cout << "graddqt\t" <<partGradbyCoeff.col(0)(0) << "\t" << partGradbyCoeff.col(0)(1) << "\t" << partGradbyCoeff.col(0)(2) << "\t" << partGradbyCoeff.col(0)(3) << "\t" <<partGradbyCoeff.col(0)(4) << "\t" << partGradbyCoeff.col(0)(5) << "\t"<< endl;
        // cout << "graddqp\t" <<partGradbyCoeff.col(1)(0) << "\t" << partGradbyCoeff.col(1)(1) << "\t" << partGradbyCoeff.col(1)(2) << "\t" << partGradbyCoeff.col(1)(3) << "\t" <<partGradbyCoeff.col(1)(4) << "\t" << partGradbyCoeff.col(1)(5) << "\t"<< endl;
    }

    inline void getPartiald2qbyCoeff(const Eigen::Vector3d &gradd2q,
                                     const Eigen::Vector2d &dAng,
                                     const Eigen::Vector2d &d2Ang,
                                     const Eigen::Vector2d &cst,
                                     const Eigen::VectorXd &beta0,
                                     const Eigen::VectorXd &beta1,
                                     const Eigen::VectorXd &beta2,
                                     Eigen::MatrixX2d &partGradbyCoeff)
    {
        Eigen::Matrix<double, 3, 5> pd2qptc, pd2qppc;
        Eigen::Vector3d pd2qpdt, pd2qpdp, pd2qpd2t, pd2qpd2p;
        Eigen::Matrix<double, 5, 1> base2;
        base2 << dAng(0) * dAng(0), dAng(0) * dAng(1), d2Ang(0), dAng(1) * dAng(1), d2Ang(1);
        pd2qptc.row(0) << -g3, 2 * g2, -g1, -g3, -g4; //
        pd2qptc.row(1) << -g4, -2 * g1, -g2, -g4, g3; //
        pd2qptc.row(2) << cst(1), 0.0, -cst(0), 0.0, 0.0;
        pd2qpdt(0) = -2 * g1 * dAng(0) - 2 * g4 * dAng(1); //
        pd2qpdt(1) = -2 * g2 * dAng(0) + 2 * g3 * dAng(1); //
        pd2qpdt(2) = -2 * cst(0) * dAng(0);                //
        pd2qpd2t << g3, g4, -cst(1);                       //

        pd2qppc.row(0) << g2, -2 * g3, -g4, g2, -g1;       //
        pd2qppc.row(1) << -g1, -2 * g4, g3, -g1, -g2;      //
        pd2qppc.row(2) << 0.0, 0.0, 0.0, 0.0, 0.0;         //
        pd2qpdp(0) = -2 * g4 * dAng(0) - 2 * g1 * dAng(1); //
        pd2qpdp(1) = 2 * g3 * dAng(0) - 2 * g2 * dAng(1);  //
        pd2qpdp(2) = 0.0;                                  //
        pd2qpd2p << -g2, g1, 0.0;                          //
        partGradbyCoeff.col(0) += gradd2q.dot(pd2qptc * base2) * beta0 + gradd2q.dot(pd2qpdt) * beta1 + gradd2q.dot(pd2qpd2t) * beta2;
        partGradbyCoeff.col(1) += gradd2q.dot(pd2qppc * base2) * beta0 + gradd2q.dot(pd2qpdp) * beta1 + gradd2q.dot(pd2qpd2p) * beta2;
        // test0 = gradd2q.dot(pd2qptc * base2) * beta0 + gradd2q.dot(pd2qpdt) * beta1 + gradd2q.dot(pd2qpd2t) * beta2;
        // test1 = gradd2q.dot(pd2qppc * base2) * beta0 + gradd2q.dot(pd2qpdp) * beta1 + gradd2q.dot(pd2qpd2p) * beta2;
        // cout << "d2q\t" << test0(0) << "\t" << test0(1) << "\t"<< test0(2) << "\t"<< test0(3) << "\t"<< test0(4) << "\t"<< test0(5) << endl;
        // cout << "d2q\t" << test1(0) << "\t" << test1(1) << "\t"<< test1(2) << "\t"<< test1(3) << "\t"<< test1(4) << "\t"<< test1(5) << endl;
        // cout << "gradd2qt\t" <<partGradbyCoeff.col(0)(0) << "\t" << partGradbyCoeff.col(0)(1) << "\t" << partGradbyCoeff.col(0)(2) << "\t" << partGradbyCoeff.col(0)(3) << "\t" <<partGradbyCoeff.col(0)(4) << "\t" << partGradbyCoeff.col(0)(5) << "\t"<< endl;
        // cout << "gradd2qp\t" <<partGradbyCoeff.col(1)(0) << "\t" << partGradbyCoeff.col(1)(1) << "\t" << partGradbyCoeff.col(1)(2) << "\t" << partGradbyCoeff.col(1)(3) << "\t" <<partGradbyCoeff.col(1)(4) << "\t" << partGradbyCoeff.col(1)(5) << "\t"<< endl;
    }

    inline void getPartiald3qbyCoeff(const Eigen::Vector3d &gradd3q,
                                     const Eigen::Vector2d &dAng,
                                     const Eigen::Vector2d &d2Ang,
                                     const Eigen::Vector2d &d3Ang,
                                     const Eigen::Vector2d &cst,
                                     const Eigen::VectorXd &beta0,
                                     const Eigen::VectorXd &beta1,
                                     const Eigen::VectorXd &beta2,
                                     const Eigen::VectorXd &beta3,
                                     Eigen::MatrixX2d &partGradbyCoeff)
    {
        Eigen::Matrix<double, 3, 10> pd3qptc, pd3qppc;
        Eigen::Vector3d pd3qpdt, pd3qpdp, pd3qpd2t, pd3qpd2p, pd3qpd3t, pd3qpd3p;
        Eigen::Matrix<double, 10, 1> base3;
        base3 << pow(dAng(0), 3), pow(dAng(0), 2) * dAng(1), dAng(0) * d2Ang(0), dAng(0) * pow(dAng(1), 2), d2Ang(0) * dAng(1),
            dAng(0) * d2Ang(1), d3Ang(0), pow(dAng(1), 3), dAng(1) * d2Ang(1), d3Ang(1);
        pd3qptc.row(0) << g1, 3 * g4, -3 * g3, 3 * g1, 3 * g2, 3 * g2, -g1, g4, -3 * g3, -g4;                                                       //
        pd3qptc.row(1) << g2, -3 * g3, -3 * g4, 3 * g2, -3 * g1, -3 * g1, -g2, -g3, -3 * g4, g3;                                                    //
        pd3qptc.row(2) << cst(0), 0.0, 3 * cst(1), 0.0, 0.0, 0.0, -cst(0), 0.0, 0.0, 0.0;                                                           //
        pd3qpdt(0) = -3 * g3 * dAng(0) * dAng(0) + 6 * g2 * dAng(0) * dAng(1) - 3 * g1 * d2Ang(0) - 3 * g3 * dAng(1) * dAng(1) - 3 * g4 * d2Ang(1); //
        pd3qpdt(1) = -3 * g4 * dAng(0) * dAng(0) - 6 * g1 * dAng(0) * dAng(1) - 3 * g2 * d2Ang(0) - 3 * g4 * dAng(1) * dAng(1) + 3 * g3 * d2Ang(1); //
        pd3qpdt(2) = 3 * cst(1) * dAng(0) * dAng(0) - 3 * cst(0) * d2Ang(0);                                                                        //
        pd3qpd2t(0) = -3 * g1 * dAng(0) - 3 * g4 * dAng(1);                                                                                         //
        pd3qpd2t(1) = -3 * g2 * dAng(0) + 3 * g3 * dAng(1);                                                                                         //
        pd3qpd2t(2) = -3 * cst(0) * dAng(0);
        pd3qpd3t << g3, g4, -cst(1);

        pd3qppc.row(0) << g4, 3 * g1, 3 * g2, 3 * g4, -3 * g3, -3 * g3, -g4, g1, 3 * g2, -g1;                                                       //
        pd3qppc.row(1) << -g3, 3 * g2, -3 * g1, -3 * g3, -3 * g4, -3 * g4, g3, g2, -3 * g1, -g2;                                                    //
        pd3qppc.row(2) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;                                                                         //
        pd3qpdp(0) = 3 * g2 * dAng(0) * dAng(0) - 6 * g3 * dAng(0) * dAng(1) - 3 * g4 * d2Ang(0) + 3 * g2 * dAng(1) * dAng(1) - 3 * g1 * d2Ang(1);  //
        pd3qpdp(1) = -3 * g1 * dAng(0) * dAng(0) - 6 * g4 * dAng(0) * dAng(1) + 3 * g3 * d2Ang(0) - 3 * g1 * dAng(1) * dAng(1) - 3 * g2 * d2Ang(1); //
        pd3qpdp(2) = 0.0;                                                                                                                           //
        pd3qpd2p(0) = -3 * g4 * dAng(0) - 3 * g1 * dAng(1);                                                                                         //
        pd3qpd2p(1) = 3 * g3 * dAng(0) - 3 * g2 * dAng(1);                                                                                          //
        pd3qpd2p(2) = 0.0;
        pd3qpd3p << -g2, g1, 0.0;
        partGradbyCoeff.col(0) += gradd3q.dot(pd3qptc * base3) * beta0 + gradd3q.dot(pd3qpdt) * beta1 + gradd3q.dot(pd3qpd2t) * beta2 + gradd3q.dot(pd3qpd3t) * beta3;
        partGradbyCoeff.col(1) += gradd3q.dot(pd3qppc * base3) * beta0 + gradd3q.dot(pd3qpdp) * beta1 + gradd3q.dot(pd3qpd2p) * beta2 + gradd3q.dot(pd3qpd3p) * beta3;
        // test0 = gradd3q.dot(pd3qptc * base3) * beta0 + gradd3q.dot(pd3qpdt) * beta1 + gradd3q.dot(pd3qpd2t) * beta2 + gradd3q.dot(pd3qpd3t) * beta3;
        // test1 = gradd3q.dot(pd3qppc * base3) * beta0 + gradd3q.dot(pd3qpdp) * beta1 + gradd3q.dot(pd3qpd2p) * beta2 + gradd3q.dot(pd3qpd3p) * beta3;
        // cout << "d3q\t" << test0(0) << "\t" << test0(1) << "\t"<< test0(2) << "\t"<< test0(3) << "\t"<< test0(4) << "\t"<< test0(5) << endl;
        // cout << "d3q\t" << test1(0) << "\t" << test1(1) << "\t"<< test1(2) << "\t"<< test1(3) << "\t"<< test1(4) << "\t"<< test1(5) << endl;
        // cout << "gradd3qt\t" <<partGradbyCoeff.col(0)(0) << "\t" << partGradbyCoeff.col(0)(1) << "\t" << partGradbyCoeff.col(0)(2) << "\t" << partGradbyCoeff.col(0)(3) << "\t" <<partGradbyCoeff.col(0)(4) << "\t" << partGradbyCoeff.col(0)(5) << "\t"<< endl;
        // cout << "gradd3qp\t" <<partGradbyCoeff.col(1)(0) << "\t" << partGradbyCoeff.col(1)(1) << "\t" << partGradbyCoeff.col(1)(2) << "\t" << partGradbyCoeff.col(1)(3) << "\t" <<partGradbyCoeff.col(1)(4) << "\t" << partGradbyCoeff.col(1)(5) << "\t"<< endl;
    }

    inline Eigen::MatrixX2d getGradCoeff(const Eigen::Vector2d &dAng,
                                         const Eigen::Vector2d &d2Ang,
                                         const Eigen::Vector2d &d3Ang,
                                         const Eigen::Vector2d &cst,
                                         const Eigen::VectorXd &beta0,
                                         const Eigen::VectorXd &beta1,
                                         const Eigen::VectorXd &beta2,
                                         const Eigen::VectorXd &beta3,
                                         const Eigen::VectorXd &totalGradQ,
                                         const Eigen::VectorXd &totalGraddQ,
                                         const Eigen::VectorXd &totalGradd2Q,
                                         const Eigen::VectorXd &totalGradd3Q)
    {
        Eigen::MatrixX2d partGradbyCoeff;
        partGradbyCoeff.resize(8, 2);
        // cout << "======================================================================"<< endl;
        getPartialqbyCoeff(totalGradQ, cst, beta0, partGradbyCoeff);
        // cout << partGradbyCoeff<< endl;
        getPartialdqbyCoeff(totalGraddQ, dAng, cst, beta0, beta1, partGradbyCoeff);
        getPartiald2qbyCoeff(totalGradd2Q, dAng, d2Ang, cst, beta0, beta1, beta2, partGradbyCoeff);
        getPartiald3qbyCoeff(totalGradd3Q, dAng, d2Ang, d3Ang, cst, beta0, beta1, beta2, beta3, partGradbyCoeff);
        // cout << partGradbyCoeff<< endl;
        return partGradbyCoeff;
    }

        inline Eigen::MatrixX2d  getGradCoeffs(const Eigen::Vector2d &dAng,
                                         const Eigen::Vector2d &d2Ang,
                                         const Eigen::Vector2d &d3Ang,
                                         const Eigen::Vector2d &d4Ang,
                                         const Eigen::Vector2d &cst,
                                         const Eigen::Vector4d &g,
                                         const Eigen::VectorXd &d2qb,
                                         const Eigen::VectorXd &d3qb,
                                         const Eigen::VectorXd &d4qb,
                                         const Eigen::VectorXd &beta0,
                                         const Eigen::VectorXd &beta1,
                                         const Eigen::VectorXd &beta2,
                                         const Eigen::VectorXd &beta3,
                                         const Eigen::VectorXd &beta4,
                                         const Eigen::Vector3d &gradq,
                                         const Eigen::Vector3d &graddq,
                                         const Eigen::Vector3d &gradd2q,
                                         const Eigen::Vector3d &gradd3q,
                                         const Eigen::Vector3d &gradd4q)
    {
        Eigen::MatrixX2d partGradbyCoeff;
        partGradbyCoeff.resize(8, 2);
        // cout << "======================================================================"<< endl;
        pqpt_(0) = g(2);
        pqpt_(1) = g(3);
        pqpt_(2) = -cst(1);
        pqpp_(0) = -g(1);
        pqpp_(1) = g(0);
        pqpp_(2) = 0.0;

        pdqpt(0) = - g(0) * dAng(0) - g(3) * dAng(1);
        pdqpt(1) = - g(1) * dAng(0) + g(2) * dAng(1);
        pdqpt(2) = -cst(0) * dAng(0);
        pdqpp(0) = - g(3) * dAng(0) - g(0) * dAng(1);
        pdqpp(1) = g(2) * dAng(0) - g(1) * dAng(1);
        pdqpp(2) = 0.0;
        pdqpdt(0) = g(2);
        pdqpdt(1) = g(3);
        pdqpdt(2) = -cst(1);
        pdqpdp(0) = - g(1);
        pdqpdp(1) = g(0);
        pdqpdp(2) = 0.0;

        pd2qpt(0) = - g(3) * d2qb(0) + 2 * g(1) * d2qb(1) - g(2) * d2qb(2) - g(0) * d2qb(3);
        pd2qpt(1) = - g(3) * d2qb(2) - g(1) * d2qb(3) + g(2) * d2qb(0) - 2 * g(0) * d2qb(1);
        pd2qpt(2) = -cst(0) * d2Ang(0) + cst(1) * pow(dAng(0), 2);
        pd2qpp(0) = - g(0) * d2qb(0) - 2 * g(2) * d2qb(1) + g(1) * d2qb(2) - g(3) * d2qb(3);
        pd2qpp(1) = - g(0) * d2qb(2) + g(2) * d2qb(3) - g(1) * d2qb(0) - 2 * g(3) * d2qb(1);
        pd2qpp(2) = 0.0;
        pd2qpdt(0) = - 2 * g(3) * dAng(1) - 2 * g(0) * dAng(0);
        pd2qpdt(1) = 2 * g(2) * dAng(1) - 2 * g(1) * dAng(0);
        pd2qpdt(2) = -2 * cst(0) * dAng(0);
        pd2qpdp(0) = - 2 * g(3) * dAng(0) - 2 * g(0) * dAng(1);
        pd2qpdp(1) = 2 * g(2) * dAng(0) - 2 * g(1) * dAng(1);
        pd2qpdp(2) = 0.0;
        pd2qpd2t(0) = g(2);
        pd2qpd2t(1) = g(3);
        pd2qpd2t(2) = -cst(1);
        pd2qpd2p(0) = - g(1);
        pd2qpd2p(1) = g(0);
        pd2qpd2p(2) = 0.0;

        pd3qpt(0) = - g(3) * d3qb(0) + g(1) * d3qb(1) - g(2) * d3qb(2) + g(0) * d3qb(3);
        pd3qpt(1) = - g(3) * d3qb(2) + g(1) * d3qb(3) + g(2) * d3qb(0) - g(0) * d3qb(1);
        pd3qpt(2) = cst(0) * d3qb(4) + 3 * cst(1) * d3qb(5);
        pd3qpp(0) = - g(0) * d3qb(0) - g(2) * d3qb(1) + g(1) * d3qb(2) + g(3) * d3qb(3);
        pd3qpp(1) = - g(0) * d3qb(2) - g(2) * d3qb(3) - g(1) * d3qb(0) - g(3) * d3qb(1);
        pd3qpp(2) = 0.0;
        pd3qpdt(0) = 6 * g(1) * d2qb(1) - 3 * g(3) * d2qb(0) - 3 * g(0) * d2qb(3) - 3 * g(2) * d2qb(2);
        pd3qpdt(1) = -3 * g(1) * d2qb(3) - 3 * g(3) * d2qb(2) - 6 * g(0) * d2qb(1) + 3 * g(2) * d2qb(0);
        pd3qpdt(2) = 3 * cst(1) * pow(dAng(0), 2) - 3 * cst(0) * d2Ang(0);
        pd3qpdp(0) = -6 * g(2) * d2qb(1) - 3 * g(0) * d2qb(0) - 3 * g(3) * d2qb(3) + 3 * g(1) * d2qb(2);
        pd3qpdp(1) = -6 * g(3) * d2qb(1) - 3 * g(1) * d2qb(0) + 3 * g(2) * d2qb(3) - 3 * g(0) * d2qb(2);
        pd3qpdp(2) = 0.0;
        pd3qpd2t(0) = - 3 * g(3) * dAng(1) - 3 * g(0) * dAng(0);
        pd3qpd2t(1) = 3 * g(2) * dAng(1) - 3 * g(1) * dAng(0);
        pd3qpd2t(2) = -3 * cst(0) * dAng(0);
        pd3qpd2p(0) = - 3 * g(0) * dAng(1) - 3 * g(3) * dAng(0);
        pd3qpd2p(1) = - 3 * g(1) * dAng(1) + 3 * g(2) * dAng(0);
        pd3qpd2p(2) = 0.0;
        pd3qpd3t(0) = g(2);
        pd3qpd3t(1) = g(3);
        pd3qpd3t(2) = -cst(1);
        pd3qpd3p(0) = - g(1);
        pd3qpd3p(1) = g(0);
        pd3qpd3p(2) = 0.0;
        
        pd4qpt(0) =  g(3) * d4qb(0) - g(1) * d4qb(1) + g(2) * d4qb(2) - g(0) * d4qb(3);
        pd4qpt(1) =  g(3) * d4qb(2) - g(1) * d4qb(3) - g(2) * d4qb(0) + g(0) * d4qb(1);
        pd4qpt(2) = cst(0) * d4qb(4) - cst(1) * d4qb(5);
        pd4qpp(0) = g(0) * d4qb(0) + g(2) * d4qb(1) - g(1) * d4qb(2) - g(3) * d4qb(3);
        pd4qpp(1) = g(0) * d4qb(2) + g(2) * d4qb(3) + g(1) * d4qb(0) + g(3) * d4qb(1);
        pd4qpp(2) = 0.0;
        pd4qpdt(0) = 4 * g(1) * d3qb(1) - 4 * g(3) * d3qb(0) + 4 * g(0) * d3qb(3) - 4 * g(2) * d3qb(2);
        pd4qpdt(1) = 4 * g(1) * d3qb(3) - 4 * g(3) * d3qb(2) - 4 * g(0) * d3qb(1) + 4 * g(2) * d3qb(0);
        pd4qpdt(2) = 4 * cst(0) * d3qb(4) + 12 * cst(1) * d3qb(5);
        pd4qpdp(0) = 4 * g(1) * d3qb(2) + 4 * g(3) * d3qb(3) - 4 * g(0) * d3qb(0) - 4 * g(2) * d3qb(1);
        pd4qpdp(1) = -4 * g(1) * d3qb(0) - 4 * g(3) * d3qb(1) - 4 * g(0) * d3qb(2) - 4 * g(2) * d3qb(3);
        pd4qpdp(2) = 0.0;
        pd4qpd2t(0) = 12 * g(1) * d2qb(1) - 6 * g(3) * d2qb(0) - 6 * g(0) * d2qb(3) - 6 * g(2) * d2qb(2);
        pd4qpd2t(1) = -6 * g(1) * d2qb(3) - 6 * g(3) * d2qb(2) - 12 * g(0) * d2qb(1) + 6 * g(2) * d2qb(0);
        pd4qpd2t(2) = 6 * cst(1) * pow(dAng(0), 2) - 6 * cst(0) * d2Ang(0);
        pd4qpd2p(0) = 6 * g(1) * d2qb(2) - 6 * g(3) * d2qb(3) - 6 * g(0) * d2qb(0) - 12 * g(2) * d2qb(1);
        pd4qpd2p(1) = -6 * g(1) * d2qb(0) - 12 * g(3) * d2qb(1) - 6 * g(0) * d2qb(2) + 6 * g(2) * d2qb(3);
        pd4qpd2p(2) = 0.0;
        pd4qpd3t(0) = -4 * g(3) * dAng(1) - 4 * g(0) * dAng(0);
        pd4qpd3t(1) = 4 * g(2) * dAng(1) - 4 * g(1) * dAng(0);
        pd4qpd3t(2) = -4 * cst(0) * dAng(0);
        pd4qpd3p(0) = - 4 * g(0) * dAng(1) - 4 * g(3) * dAng(0);
        pd4qpd3p(1) = - 4 * g(1) * dAng(1) + 4 * g(2) * dAng(0);
        pd4qpd3p(2) = 0.0;
        pd4qpd4t(0) = g(2);
        pd4qpd4t(1) = g(3);
        pd4qpd4t(2) = -cst(1);
        pd4qpd4p(0) = - g(1);
        pd4qpd4p(1) = g(0);
        pd4qpd4p(2) = 0.0;


        partGradbyCoeff.col(0) = (gradq.dot(pqpt_) + graddq.dot(pdqpt) + gradd2q.dot(pd2qpt) + gradd3q.dot(pd3qpt) + gradd4q.dot(pd4qpt)) * beta0 
                                  + (graddq.dot(pdqpdt) + gradd2q.dot(pd2qpdt) + gradd3q.dot(pd3qpdt) + gradd4q.dot(pd4qpdt)) * beta1
                                  + (gradd2q.dot(pd2qpd2t) + gradd3q.dot(pd3qpd2t) + gradd4q.dot(pd4qpd2t)) * beta2
                                  + (gradd3q.dot(pd3qpd3t) + gradd4q.dot(pd4qpd3t)) * beta3
                                  + (gradd4q.dot(pd4qpd4t)) * beta4;
        partGradbyCoeff.col(1) = (gradq.dot(pqpp_) + graddq.dot(pdqpp) + gradd2q.dot(pd2qpp) + gradd3q.dot(pd3qpp) + gradd4q.dot(pd4qpp)) * beta0 
                                  + (graddq.dot(pdqpdp) + gradd2q.dot(pd2qpdp) + gradd3q.dot(pd3qpdp) + gradd4q.dot(pd4qpdp)) * beta1
                                  + (gradd2q.dot(pd2qpd2p) + gradd3q.dot(pd3qpd2p) + gradd4q.dot(pd4qpd2p)) * beta2
                                  + (gradd3q.dot(pd3qpd3p) + gradd4q.dot(pd4qpd3p)) * beta3
                                  + (gradd4q.dot(pd4qpd4p)) * beta4;


        // cout << partGradbyCoeff<< endl;
        return partGradbyCoeff;
    }

    Eigen::Vector3d quat_to_rpy(const Eigen::Quaterniond &q)
    {
        Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);

        Eigen::Vector3d rpy;
        rpy[0] = eulerAngle[2] * 180.0 / M_PI;
        rpy[1] = eulerAngle[1] * 180.0 / M_PI;
        rpy[2] = eulerAngle[0] * 180.0 / M_PI;

        if (std::fabs(rpy.y()) >= 90.0)
        {
            rpy.y() = 180.0 - rpy.y();
            rpy.x() += 180.0;
            rpy.z() += 180.0;

            for (int i = 0; i <= 2; i++)
            {
                for (int cnt = 0; cnt <= 3; cnt++)
                {
                    if (rpy[i] > 180.0)
                    {
                        rpy[i] -= 360.0;
                    }
                    if (rpy[i] < -180.0)
                    {
                        rpy[i] += 360.0;
                    }
                }
            }
        }
        return rpy;
    }
}