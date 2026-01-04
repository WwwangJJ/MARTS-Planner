#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/funs.hpp"
#include "gcopter/trajectory.hpp"
#include "plan_env/grid_map.h"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std;
namespace gcopter
{

    class GCOPTER_PolytopeSFC
    {
    public:

    private:
        minco::MINCO_S4NU minco;
        flatness::FlatnessMap flatmap;

        double rho, rhoE, scale;
        Eigen::Matrix<double, 3, 4> headPVAL, tailPVAL;
        Eigen::Matrix4Xd headPVAQ, tailPVAQ;
        Eigen::Matrix3Xd initPath, initWayVels, initWayAccs;
        Eigen::Vector3d  inifinAngT;
        Eigen::VectorXd  angTBs;  

        int pieceN, droneN;
        int spatialDimL, spatialDimQ, temporalDim, continuityDesDim, boundaryCondSlackDim;

        double smoothFactor;
        int integralRes;
        int iter_num_;
        bool isPrint;
        Eigen::Matrix4d allocM, allocMI;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        Eigen::VectorXd physicalPm;
        Eigen::Vector3d continuityDes;
        Eigen::Vector3d boundaryCondSlack;
        double allocSpeed;
        double disBetweenWaypoints;
        Eigen::VectorXi fixPtsNums, fixVelsNums, fixAccsNums;
        lbfgs::lbfgs_parameter_t lbfgs_params;
        Eigen::VectorXd lambdas;
        Eigen::MatrixX3d points, gradByPoints, gradByVs, gradByAs;
        Eigen::VectorXd partialGradByCoeffs;
        Eigen::VectorXd angTs, gradByAngTs, gradBydAngTs, gradByd2AngTs;
        std::vector<Eigen::Vector3d> gradQ;
        std::vector<double> gradF;
        Eigen::VectorXd times, gradByTimes, partialGradByTimes, times_, gradByTimes_;
        Eigen::VectorXi intpNum;

        std::vector<Eigen::Vector3d> qs, angT;
        std::vector<Eigen::Vector2d> cst, csp;
        std::vector<Eigen::Vector4d> gs;
        std::vector<Eigen::VectorXd> rpm1, rpm2, rpm3, rpm4, accQs;
        Eigen::VectorXd accs;
        GridMap::Ptr gridMap;

    public:

        inline std::vector<Eigen::VectorXd> getRPM1()
        {
            return rpm1;
        }

        inline std::vector<Eigen::VectorXd> getRPM2()
        {
            return rpm2;
        }

        inline std::vector<Eigen::VectorXd> getRPM3()
        {
            return rpm3;
        }

        inline std::vector<Eigen::VectorXd> getRPM4()
        {
            return rpm4;
        }

        inline Eigen::VectorXd &getAcc()
        {
            return accs;
        }

        inline std::vector<Eigen::VectorXd> &getAccQs()
        {
            return accQs;
        }

    private:
        static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)    // from opt variable taus to real piece time Ts
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0) : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)   // from real piece time Ts to opt variable taus  
         {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)         // frome real piece time T's grads to opt variable tau's grad 
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }
            return;
        }

        static inline Eigen::Vector3d getAngT(const Eigen::Vector3d &lam, 
                                              const Eigen::VectorXd &angTBounds)
        {
            double l2;
            Eigen::Vector3d angT;
            for (int i = 0; i < 3; i++)
            {
                l2 = pow(lam(i), 2);
                angT(i) = angTBounds(1 + 2 * i) + 4.0 * (angTBounds(0 + 2 * i) - angTBounds(1 + 2 * i)) * l2 / pow(l2 + 1.0, 2);
            }
            return angT;
        }

        static inline void forwardP(const Eigen::VectorXd &xiL,
                                    const Eigen::VectorXd &xiQ,
                                    Eigen::MatrixX3d &P, 
                                    Eigen::VectorXd &angTsV,
                                    const int &pieceN,
                                    const int &droneN,
                                    const Eigen::VectorXd angTBounds)       
        {
            Eigen::Map<const Eigen::MatrixXd> qs(xiL.data(), pieceN - 1, 3);
            Eigen::Map<const Eigen::MatrixXd> alams(xiQ.data(), pieceN - 1, 3 * droneN);
            P.resize(pieceN - 1, 3);
            angTsV.resize((pieceN - 1) * (3 * droneN));
            Eigen::Map<Eigen::MatrixXd> angTsM(angTsV.data(), pieceN - 1, 3 * droneN);
            Eigen::VectorXd q;
            // cout << angTBounds(0) /M_PI<< "\t" <<  angTBounds(1)/M_PI << "\t" << angTBounds(2)/M_PI << "\t" << angTBounds(3)/M_PI << "\t" << angTBounds(4) << "\t" << angTBounds(5) << endl;
            for (int i = 0; i < pieceN - 1; i++)
            {
                P.row(i) = qs.row(i);
                for (int j = 0; j < droneN; j++)
                {
                    angTsM.row(i).segment<3>(j * 3) = getAngT(alams.row(i).segment<3>(j * 3), angTBounds);
                }
            }
            // cout << P << endl;
            // cout << angTsM / M_PI<< endl;
            return;
        }

        static inline void backwardP(const Eigen::MatrixX3d &pts,
                                     const Eigen::Vector3d &angT,   // initAngT
                                     Eigen::VectorXd &x,
                                     const Eigen::VectorXd angTBounds,
                                     const int pieceN,
                                     const int droneN)
        {
            Eigen::Map<Eigen::MatrixX3d> qs(x.data(), pieceN - 1, 3);
            Eigen::Map<Eigen::MatrixXd> alams(x.data() + 3 * (pieceN - 1), pieceN - 1, 3 * droneN);
            Eigen::VectorXd lam(3), lams(3 * droneN);
            double c;
            for (int i = 0; i < 3; i++)
            {
                if (angTBounds(1 + i * 2) <= angT(i) && angT(i) <= angTBounds(0 + i * 2))
                {
                    c = 4.0 * (angTBounds(0 + i * 2) - angTBounds(1 + i * 2)) / (angT(i) - angTBounds(1 + i * 2));
                    lam(i) = sqrt(max(0.5 * ((c - 2.0) - sqrt(c * c - 4.0 * c)), 0.0));
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("gcopter"), "angT overstep the boundary!");
                }
            }
            for (int i = 0; i < droneN; i++)
            {
                lams.segment<3>(i * 3) = lam;
            }
            for (int i = 0; i < pieceN - 1; i++)
            {
                qs.row(i) = pts.row(i);
                alams.row(i) = lams;
            }
            // cout << qs << endl;
            // cout << alams << endl;
            // cout << x << endl;
            return;
        }

        static inline Eigen::Vector3d getGradLambdas(const Eigen::Vector3d &gdAngT, 
                                                     const Eigen::Vector3d &lams, 
                                                     const Eigen::VectorXd &angTBounds)
        {
            double l;
            Eigen::Vector3d gdlams;
            for (int i = 0; i < 3; i++)
            {
                l = lams(i);
                gdlams(i) = gdAngT(i) * 8.0 * (angTBounds(i * 2 + 0) - angTBounds(i * 2 + 1)) * l * (1.0 - l * l) / pow(l * l + 1.0, 3);             
            }
            return gdlams;  
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xiLV,
                                         const Eigen::VectorXd &xiQV,
                                         EIGENVEC &gradXiLV,
                                         EIGENVEC &gradXiQV,
                                         const Eigen::MatrixX3d &gradP,
                                         const Eigen::VectorXd &gradAngTV,
                                         const Eigen::VectorXd angTBounds,
                                         const int &pieceN,
                                         const int &droneN)
        {
            gradXiLV.resize((pieceN - 1) * 3);
            gradXiQV.resize((pieceN - 1) * (3 * droneN));
            Eigen::Map<Eigen::MatrixX3d> gradXiLM(gradXiLV.data(), pieceN - 1, 3);
            Eigen::Map<Eigen::MatrixXd> gradXiQM(gradXiQV.data(), pieceN - 1, 3 * droneN);
            Eigen::Map<const Eigen::MatrixX3d> xiLM(xiLV.data(), pieceN - 1, 3);
            Eigen::Map<const Eigen::MatrixXd> xiQM(xiQV.data(), pieceN - 1, 3 * droneN);
            Eigen::Map<const Eigen::MatrixXd> gradAngTM(gradAngTV.data(), pieceN - 1, 3 * droneN);
            for (int i = 0; i < pieceN - 1; i++)
            {
                gradXiLM.row(i) = gradP.row(i);
                for (int j = 0; j < droneN; j++)
                {
                    gradXiQM.row(i).segment(j * 3, 3) = getGradLambdas(gradAngTM.row(i).segment(j * 3, 3), xiQM.row(i).segment(j * 3, 3), angTBounds);
                }
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xiQ,
                                               double &cost,
                                               EIGENVEC &gradXiQ)
        {
            const int sizeQ = xiQ.size();
            gradXiQ.resize(xiQ.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            double kk = 10000.0;
            for (int i = 0; i < sizeQ; i++)
            {
                sqrNormQ = pow(xiQ(i), 2);
                sqrNormViolation = kk * (sqrNormQ - 1.0);
                // if (sqrNormViolation > 0.0)
                // {
                //     c = sqrNormViolation * sqrNormViolation;
                //     dc = 3.0 * c;
                //     c *= sqrNormViolation;
                //     cost += c;
                //     gradXiQ(i) += dc * 2.0 * xiQ(i);
                // }
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation;
                    cost += c;
                    gradXiQ(i) += kk *  2.0 * xiQ(i);
                }
            }

            return;
        }

        static inline void getGradByTimes_(Eigen::VectorXd &gradTimes,
                                           Eigen::VectorXd &gradTimes_,
                                           const Eigen::VectorXi &intp)
        {
            int k = 0;
            gradTimes_.resize(gradTimes.size());
            gradTimes_.setZero();
            for (int i = 0; i < intp.size(); i++)
            {
                for (int j = 0; j < intp(i) + 1; j++)
                {
                   gradTimes_(i) += gradTimes(k + j);    
                }
                k += intp(i) + 1;
            }
        }

        inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                            const Eigen::MatrixXd &coeffsM,
                                            flatness::FlatnessMap &flatMap,
                                            Eigen::VectorXd &cost,
                                            Eigen::VectorXd &gradT,
                                            Eigen::VectorXd &gradC)
        {
            const double disSqrMax = magnitudeBd(7) * magnitudeBd(7);
            const double clearence = magnitudeBd(9);
            const double RPM2Mean = 0.5 * magnitudeBd(10) * magnitudeBd(10);
            const double RPM2Radi = RPM2Mean;
            const double RPM2SqrRadi = RPM2Radi * RPM2Radi;

            const double weightDis = penaltyWt(0);
            const double weightRPM = penaltyWt(1);
            const double weightForce = penaltyWt(2);
          
            Eigen::Vector3d pos, vel, acc, jer, sna, cak, posQ, velQ, accQ, jerQ, snaQ, dq, d2q, d3q, d4q, d5q, dAngT, d2AngT, d3AngT, d4AngT, d5AngT;
            Eigen::Vector3d totalGradPos, totalGradAcc, totalGradJer, totalGradSna;
            Eigen::Vector4d ftau, rpm2s, quat;
            Eigen::VectorXd d2qb, d5qb, d3qb, d4qb;
            Eigen::Matrix<bool, 4, 1>  isRPMGrads;
            Eigen::Vector4d gradRPM2s, gradftau;
            Eigen::Vector3d gradPosL, gradVelL, gradAccL;
            Eigen::Vector3d totalGradQ, totalGraddQ, totalGradd2Q, totalGradd3Q, totalGradd4Q, gradPosQ, totalGraddFs, dFs, dFs1;
            double step, alpha;
            double thr;
            double s1, s2, s3, s4, s5, s6, s7;
            Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5;
            Eigen::Vector3d totalForce, loadForce, omg;
            int K, L;
            bool isCalFlatness;
            Eigen::Vector4d violaRPM2s; 
            double violaPosL, violaPosQ, violaForceQ, violaDisQ;
            double violaPosLPenaD, violaDisQPenaD, violaPosQPenaD, violaForceQPenaD, violaRPM2sPena;
            double violaPosLPena, violaDisQPena, violaPosQPena, violaForceQPena, violaRPM2sPenaD;
            double node, penaL, penaQ, dist;

            const int pieceNum = pieceN;
            const double integralFrac = 1.0 / integralRes;
            const double droneNFrac = 1.0 / droneN;
            d2qb.resize(4);
            d3qb.resize(6);
            d4qb.resize(6);
            d5qb.resize(4);
            // cout << "FFFFFF\t" << coeffsM.size() << "\t" << 6 * pieceN<< "\t" << 3 + 3 * droneN << endl;
            // Eigen::Map<const Eigen::MatrixXd> coeffsM(coeffsV.data(), 6 * pieceN, 3 + 3 * droneN);
            Eigen::Map<Eigen::MatrixXd> gradCM(gradC.data(), 8 * pieceN, 3 + 3 * droneN);
            // cout << "============================================================================================="<< endl;
            // cout << "gra\t" << physicalPm(1) << "\t" << scale << endl;
            for (int i = 0; i < pieceNum; i++)
            {
                for (int j = 0; j <= integralRes; j++)
                {
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    s6 = s3 * s3;
                    s7 = s4 * s3;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
                    beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0, beta5(4) = 0.0, beta5(5) = 120.0, beta5(6) = 720.0 * s1, beta5(7) = 2520.0 * s2;

                    step = T(i) * integralFrac;
                    const Eigen::Matrix<double, 8, 3> &cL = coeffsM.block<8, 3>(i * 8, 0);
                    pos = cL.transpose() * beta0;
                    vel = cL.transpose() * beta1;
                    acc = cL.transpose() * beta2;
                    jer = cL.transpose() * beta3;
                    sna = cL.transpose() * beta4;
                    cak = cL.transpose() * beta5;

                    totalForce.setZero();
                    totalGradPos.setZero();
                    totalGradAcc.setZero();
                    totalGradJer.setZero();
                    totalGradSna.setZero();
                    gradPosL.setZero();
                    gradAccL.setZero();
                    loadForce = acc;
                    loadForce(2) += physicalPm(1); 
                    loadForce *= physicalPm(7);
                    accs(i * integralRes + j) = acc.norm() / 10;
                    // cout <<physicalPm(7) << endl;
                    // cout << loadForce(0) << "\t" << loadForce(1) << "\t" << loadForce(2) << endl;
                    for (int k = 0; k < droneN; k++)
                    {
                        const Eigen::Matrix<double, 8, 3> &cQ = coeffsM.block(i * 8, 3 + 3 * k, 8, 3);
                        angT[k] = cQ.transpose() * beta0;
                        
                        angT[k](1) += 2 * k * M_PI / droneN;
                        cst[k] = getcs(angT[k](0));
                        csp[k] = getcs(angT[k](1));
                        qs[k] = getQ(cst[k], csp[k], gs[k]);  
                        // cout << i << "\t" << j << "\t" << k << "\t" << qs[k](0) << "\t" << qs[k](1) << "\t" << qs[k](2) << endl;  
                        // cout << i << "\t" << j << "\t" << k << "\t" << angT[k](0) * 180.0 / M_PI<< "\t" << angT[k](1) * 180.0/ M_PI << "\t" << angT[k](2) << endl;
                        totalForce += scale * angT[k](2) * qs[k]; 
                        gradQ[k].setZero();
                        gradF[k] = 0.0;
                    }
                    // cout<< "loadForce\t"<<  loadForce(0) << "\t" << loadForce(1) << "\t" << loadForce(2) << endl;

                    node = (j == 0 || j == integralRes) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    violaForceQ = (totalForce - loadForce).squaredNorm();
                    penaQ = 0.0;
                    if (smoothedL1(violaForceQ, smoothFactor, violaForceQPena, violaForceQPenaD))
                    {
                        for (int k = 0; k < droneN; k++)
                        {
                            gradQ[k] += weightForce * violaForceQPenaD * 2.0 * scale * angT[k](2) * (totalForce - loadForce);
                            gradF[k] += weightForce * violaForceQPenaD * 2.0 * (totalForce - loadForce).dot(qs[k]);
                        }
                        gradAccL += -weightForce * violaForceQPenaD * 2.0 * physicalPm(7) * (totalForce - loadForce);
                        penaQ += weightForce * violaForceQPena;
                        gradT(i) += node * integralFrac * penaQ;
                        cost(3) += node * step * (weightForce * violaForceQPena);
                    }


                    for (int k = 0; k < droneN; k++)
                    {
                        // setg(gs[k]);
                        const Eigen::Matrix<double, 8, 3> &cQ = coeffsM.block(i * 8, 3 + 3 * k, 8, 3);
                        dAngT = cQ.transpose() * beta1;
                        d2AngT = cQ.transpose() * beta2;
                        d3AngT = cQ.transpose() * beta3;
                        d4AngT = cQ.transpose() * beta4;
                        d5AngT = cQ.transpose() * beta5;
                        getd5Qs(dAngT.head(2), d2AngT.head(2), d3AngT.head(2), d4AngT.head(2), d5AngT.head(2), cst[k], csp[k], gs[k], d2qb, d3qb, d4qb, d5qb, dq, d2q, d3q, d4q, d5q);

                        posQ = pos + physicalPm(6) * qs[k];
                        velQ = vel + physicalPm(6) * dq;
                        accQ = acc + physicalPm(6) * d2q;
                        jerQ = jer + physicalPm(6) * d3q;
                        snaQ = sna + physicalPm(6) * d4q;

                        // cout << "============================================================================================="<< endl;
                        // cout << "accQ\t" << acc(0) << "\t" << acc(1) << "\t" << acc(2) << endl;
                        // cout << "d2q\t" << d2q(0) << "\t" << d2q(1) << "\t" << d2q(2) << endl;
                        // dFs << scale * angT[k](2), scale * dAngT(2), scale * d2AngT(2);
                        // dFs1 << 0.0, 1.0, 2.0;
                        // cout << dFs1(0) << "\t"  << dFs1(1) << "\t" << dFs1(2) << endl;
                        flatMap.Forward(accQ, jerQ, snaQ, qs[k], dq, d2q, dFs, 0.0, 0.0, 0.0, ftau, thr, quat, omg);

                        rpm2s = allocMI * ftau;
                        // cout <<i * integralRes + j << "\t thr \t" << k << "\t"  << thr << "\t" <<endl;
                        if ((i == pieceNum - 1 )|| j != integralRes)
                        {
                            // cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" <<endl;
                            accQs[k](i * integralRes + j) = sqrt(accQ.squaredNorm());
                            rpm1[k](i * integralRes + j) = sqrt(fabs(rpm2s[0]));
                            rpm2[k](i * integralRes + j) = sqrt(fabs(rpm2s[1]));
                            rpm3[k](i * integralRes + j) = sqrt(fabs(rpm2s[2]));
                            rpm4[k](i * integralRes + j) = sqrt(fabs(rpm2s[3]));
                            // cout << "rpm1\t" << rpm1[k](i * integralRes + j) * 10000 << "\trpm2\t" << rpm2[k](i * integralRes + j) * 10000 << "\trpm3\t" << rpm3[k](i * integralRes + j) * 10000 << "\trpm4\t" << rpm4[k](i * integralRes + j) * 10000 <<endl; 
                        }
                        penaQ = 0.0;
                        gridMap->evaluateEDT(posQ, dist);
                        violaDisQ = clearence - dist;
                        // if (smoothedL1(violaDisQ, smoothFactor, violaDisQPena, violaDisQPenaD))
                        // {
                        //     gridMap->evaluateFirstGrad(posQ, gradPosQ);
                        //     gradQ[k] += -weightDis * violaDisQPenaD * physicalPm(6) * gradPosQ;
                        //     gradPosL += -weightDis * violaDisQPenaD * gradPosQ;
                        //     penaQ += weightDis * violaDisQPena;
                        //     cost(4) += node * step * (weightDis * violaDisQPena);
                        // }

                        for (int m = k + 1; m < droneN; m++)
                        {
                            violaDisQ = disSqrMax - (physicalPm(6) * (qs[k] - qs[m])).squaredNorm();
                            if (smoothedL1(violaDisQ, smoothFactor, violaDisQPena, violaDisQPenaD))
                            {
                                gradQ[k] += -weightDis * violaDisQPenaD * 2.0 * pow(physicalPm(6), 2) * (qs[k] - qs[m]);
                                gradQ[m] += weightDis * violaDisQPenaD * 2.0 * pow(physicalPm(6), 2) * (qs[k] - qs[m]);
                                penaQ += weightDis * violaDisQPena;
                                cost(4) += node * step * (weightDis * violaDisQPena);
                            }
                        }
                        

                        gradRPM2s.setZero();
                        violaRPM2s = (rpm2s.array() - RPM2Mean).pow(2) - RPM2SqrRadi;
                        for (int m = 0; m < 4; m++)
                        {                        
                            isRPMGrads(m,0) = smoothedL1(violaRPM2s(m), smoothFactor, violaRPM2sPena, violaRPM2sPenaD);   
                            if (isRPMGrads(m,0))
                            {
                                gradRPM2s(m) = weightRPM * violaRPM2sPenaD * 2.0 * (rpm2s(m) - RPM2Mean);
                                penaQ += weightRPM * violaRPM2sPena;
                                cost(5 + m) += node * step * (weightRPM * violaRPM2sPena);
                            } 
                        }
                        
                        gradftau = allocMI.transpose() * gradRPM2s;
                        
                        isCalFlatness = (isRPMGrads(0, 0) || isRPMGrads(1, 0) || isRPMGrads(2, 0) || isRPMGrads(3, 0)); 
                        // if (isCalFlatness)
                        // {
                        //     cout << "rpm2\t" << rpm2s(0) << "\t" << rpm2s(1) << "\t" << rpm2s(2) << "\t" << rpm2s(3) << endl;
                        //     cout << "grad\t" <<  gradRPM2s(0) << "\t" << gradRPM2s(1) << "\t" << gradRPM2s(2) << "\t" << gradRPM2s(3) << endl;
                        // }
                        // cout << isRPMGrads(0, 0) << "\t" << isRPMGrads(1, 0) << "\t" << isRPMGrads(2, 0) <<"\t" << isRPMGrads(3, 0) << "\t" << isCalFlatness << endl;
                        // cout << i * integralRes + j << "\tdQ\t" << gradQ(0) << "\t" << gradQ(1) << "\t" << gradQ(2)<< endl;
                        // cout << i * integralRes + j << "\tddQ\t" << graddQ(0) << "\t" << graddQ(1) << "\t" << graddQ(2)<< endl;
                        // // cout << i * integralRes + j << "\tddQ\t" << gradOmg(0) << "\t" << gradOmg(1) << "\t" << gradOmg(2)<< endl;
                        // // cout << i * integralRes + j << "\tdQuat\t" << gradQuat(0) << "\t" << gradQuat(1) << "\t" << gradQuat(2) << "\t" << gradQuat(3)<< endl;
                        // cout << i * integralRes + j << "\tdF\t" << gradF << "\t"  << graddF <<  endl;
                        flatMap.Backward(isCalFlatness, gradQ[k], gradF[k], gradftau, totalGradQ, totalGraddQ, totalGradd2Q, totalGradd3Q, totalGradd4Q,
                                         totalGraddFs, totalGradAcc, totalGradJer, totalGradSna);

                        // cout << i * integralRes + j << "\ttdQ\t "<< totalGradQ(0) << "\t" << totalGradQ(1) << "\t" << totalGradQ(2) << endl;
                        // cout << i * integralRes + j << "\ttddQ\t "<<  totalGraddQ(0) << "\t" << totalGraddQ(1) << "\t" << totalGraddQ(2) << endl;
                        // // cout << totalGradd2Q(0) << "\t" << totalGradd2Q(1) << "\t" << totalGradd2Q(2) << endl;
                        // // cout << totalGradd3Q(0) << "\t" << totalGradd3Q(1) << "\t" << totalGradd3Q(2)  << endl;
                        // cout << i * integralRes + j << "\ttdF\t" << totalGradF << "\t" << totalGraddF<< endl;
                        // cout << totalGradAcc(0) << "\t" << totalGradAcc(1) << "\t" << totalGradAcc(2) << endl;
                        // cout << totalGradJer(0) << "\t" << totalGradJer(1) << "\t" << totalGradJer(2)<< endl;

                        gradCM.block<8, 2>(i * 8, 3 + 3 * k) += node * step * getGradCoeffs(dAngT.head(2), d2AngT.head(2), d3AngT.head(2), d4AngT.head(2), cst[k], gs[k],
                                                                                           d2qb, d3qb, d4qb, beta0, beta1, beta2, beta3, beta4, 
                                                                                           totalGradQ, totalGraddQ, totalGradd2Q, totalGradd3Q, totalGradd4Q);
                        gradCM.block<8, 1>(i * 8, 3 + 3 * k + 2) += node * step * (beta0 * totalGraddFs(0) + beta1 * totalGraddFs(1) + beta2 * totalGraddFs(2)) * scale;
                        gradT(i) += (totalGradQ.dot(dq) + totalGraddQ.dot(d2q) + totalGradd2Q.dot(d3q) + totalGradd3Q.dot(d4q) + totalGradd4Q.dot(d5q)) * alpha * node * step + 
                                 scale * (totalGraddFs(0) * dAngT(2) + totalGraddFs(1) * d2AngT(2) + totalGraddFs(2) * d3AngT(2)) * alpha * node * step +
                                  + node * integralFrac * penaQ;
                        // cout << gradT << endl; 
                        // cost += node * step * penaQ;
                    }
                    
                    // cout << totalGradPos << "\t"  <<  totalGradVel <<"\t" <<  totalGradAcc << "\t"  <<  totalGradJer << endl;
                    // cout << cost << endl;

                    totalGradPos += gradPosL;
                    totalGradAcc += gradAccL;
                    gradCM.block<8, 3>(i * 8, 0) += node * step * (beta0 * totalGradPos.transpose() + beta2 * totalGradAcc.transpose() + beta3 * totalGradJer.transpose() + beta4 * totalGradSna.transpose());
                    gradT(i) += (totalGradPos.dot(vel) + totalGradAcc.dot(jer) + totalGradJer.dot(sna) + totalGradSna.dot(cak)) * alpha * node * step;


                    // cout << "ASD\t" << gradT<< endl;
                    // cost += node * step * penaL;
                }
            }
            // cout <<gradCM << endl;

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXiQ = obj.spatialDimQ;
            const int dimXiL = obj.spatialDimL;
            const int dimXiContinuityDesDim = obj.continuityDesDim;
            const int dimXiBoundarySlackDim = obj.boundaryCondSlackDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data() + 3 * dimXiL + 3 * dimXiQ + dimXiContinuityDesDim + dimXiBoundarySlackDim, dimTau);
            Eigen::Map<const Eigen::VectorXd> xiL(x.data(), dimXiL);
            Eigen::Map<const Eigen::VectorXd> xiQ(x.data() + dimXiL, dimXiQ);
            Eigen::Map<const Eigen::MatrixX3d> xiVL(x.data() + dimXiL + dimXiQ, obj.pieceN - 1, 3);
            Eigen::Map<const Eigen::VectorXd> xiVQ(x.data() + 2 * dimXiL + dimXiQ, dimXiQ);
            Eigen::Map<const Eigen::MatrixX3d> xiAL(x.data() + 2 * dimXiL + 2 * dimXiQ, obj.pieceN - 1, 3);
            Eigen::Map<const Eigen::VectorXd> xiAQ(x.data() + 3 * dimXiL + 2 * dimXiQ, dimXiQ);
            Eigen::Map<const Eigen::VectorXd> xiCD(x.data() + 3 * dimXiL + 3 * dimXiQ, dimXiContinuityDesDim);
            Eigen::Map<const Eigen::VectorXd> xiBS(x.data() + 3 * dimXiL + 3 * dimXiQ + dimXiContinuityDesDim, dimXiBoundarySlackDim);

            Eigen::Map<Eigen::VectorXd> gradTau(g.data() + 3 * dimXiL + 3 * dimXiQ + dimXiContinuityDesDim + dimXiBoundarySlackDim, dimTau);
            Eigen::Map<Eigen::VectorXd> gradXiL(g.data(), dimXiL);
            Eigen::Map<Eigen::VectorXd> gradXiQ(g.data() + dimXiL, dimXiQ);

            forwardT(tau, obj.times_);
            getAllTimes(obj.times, obj.times_, obj.intpNum);
            // cout << "time\t" << x(dim - 1) << "\t" << obj.time << endl;
            // cout << x << endl;
            // cout << xiQ << endl;
            forwardP(xiL, xiQ, obj.points, obj.angTs, obj.pieceN, obj.droneN, obj.angTBs);            
            // Eigen::Map<const Eigen::MatrixXd> test(obj.angTs.data(), obj.pieceN - 1, 3 * obj.droneN);
            // cout << obj.points << endl;
            // cout << test / M_PI<< endl;
            Eigen::VectorXd cost;
            cost.resize(10);
            cost.setZero();
            obj.minco.setParameters(obj.points, obj.angTs, xiVL, xiVQ, xiAL, xiAQ, xiCD, xiBS, obj.times);
            obj.minco.getEnergy(cost(0), cost(1));
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            obj.attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                        obj.flatmap, cost,
                                        obj.partialGradByTimes, obj.partialGradByCoeffs);
            // cout << cost << endl;
            // cout << "grad time1\t" << obj.gradByTime << endl;
            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByAngTs, obj.gradByTimes, g, dimXiL, dimXiQ, dimXiContinuityDesDim);

            cost(2) += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;
            // obj.gradByTimes.setZero();
            getGradByTimes_(obj.gradByTimes, obj.gradByTimes_, obj.intpNum);
            // cost(1) = 0.0;
            // obj.gradByTime = 0.0;

            // cout << obj.gradByPoints << endl;

            // cost(1) += weightT * obj.time;
            // obj.gradByTime += weightT;
            // cout << "grad time4\t" << obj.gradByTime << endl;
            // obj.gradByTime = 0.0;
            backwardGradT(tau, obj.gradByTimes_, gradTau);
            // cout << "gradT\t" << obj.gradByTime << "\t" << g(dim - 1) << endl;
            // cout << g << endl;
            backwardGradP(xiL, xiQ, gradXiL, gradXiQ, obj.gradByPoints, obj.gradByAngTs, obj.angTBs, obj.pieceN, obj.droneN);
            normRetrictionLayer(xiQ, cost(9), gradXiQ);
            obj.iter_num_ +=1;
            // cout << "grad time2\t" << obj.gradByTime << endl;
            // normRetrictionLayer(xiQ, cost, gradXiQ, obj.pieceN, obj.droneN);
            // cout << obj.iter_num_ << "\tEL\t" << setw(6) << cost(0) << "\tEQ\t" << setw(6) <<  cost(1) << "\tTime\t" << setw(6) << cost(2) <<  "\tnorm\t" << setw(6) << cost(9) << endl;
            // cout << obj.iter_num_ << "\tForce\t" << setw(6) << cost(3) << "\tCollision\t" << setw(6) <<  cost(4) << endl; 
            // cout << obj.iter_num_ << "\tr1\t" << setw(6) << cost(5)  <<"\tr2\t" << setw(6) <<  cost(6) << "\tr3\t" << setw(6) <<  cost(7) << "\tr4\t" << setw(6) << cost(8) << endl; 
            // cout << obj.iter_num_ << "\tsum\t" << setw(6) <<  cost.sum() <<  endl;   
            return cost.sum();
        }

        static inline void getInitPath(const double &disBetweenWaypoints,
                                       const Eigen::Vector3d &iniPoint,
                                       const Eigen::Vector3d &finPoint,
                                       const Eigen::Matrix3Xd &wayPts,
                                       const Eigen::Matrix3Xd &wayVels,
                                       const Eigen::Matrix3Xd &wayAccs,
                                       const Eigen::VectorXi  &velNums,
                                       const Eigen::VectorXi  &accNums,
                                       Eigen::Matrix3Xd &path,
                                       Eigen::VectorXi  &fixPointsNums,
                                       Eigen::VectorXi  &fixVelsNums,
                                       Eigen::VectorXi  &fixAccsNums,
                                       Eigen::VectorXi  &intp)
        {
            const int n = wayPts.cols();
            int k = 0;
            Eigen::VectorXd dis;
            Eigen::Matrix3Xd allWayPts, disV;
            allWayPts.resize(3, n + 2);
            intp.resize(n + 1);
            fixPointsNums.resize(n);
            fixVelsNums.resize(wayVels.cols());
            fixAccsNums.resize(wayAccs.cols());
            // cout << velNums.size() << "\t" << accNums.size() << endl;
            disV.resize(3, n + 1);
            allWayPts.leftCols<1>() = iniPoint;
            allWayPts.rightCols<1>() = finPoint;
            allWayPts.block(0, 1, 3, n) = wayPts;
            disV = allWayPts.rightCols(n + 1) - allWayPts.leftCols(n + 1);
            for (int i = 0; i < n + 1; i++)
            {
                intp(i) = max(int(ceil(disV.col(i).norm() / disBetweenWaypoints)) - 1, 0); 
                // cout <<intp(i) << "\t" << disV.col(i).norm() << "\t" << disBetweenWaypoints <<endl;
            }

            path.resize(3, intp.sum() + n + 2);
            // cout <<"1111111\t" << n << "\t" << intp.sum() + n + 2 << endl;
            path.leftCols<1>() = iniPoint;
            for (int i = 0; i < n + 1; i++)
            {
                for (int j = 0; j < intp(i); j++)
                {
                    // cout << k + j + 1<< endl;
                    path.col(k + j + 1) = path.col(k) + disV.col(i) * (j + 1) / (intp(i) + 1);
                }
                path.col(k + intp(i) + 1) = allWayPts.col(i + 1);
                k += intp(i) + 1;
            }
            k = 0;
            for (int i = 0; i < n; i++)
            {
                fixPointsNums(i) = k + intp(i);
                k += intp(i) + 1; 
            }

            for (int i = 0; i < velNums.size(); i++)
            {
                fixVelsNums(i) = fixPointsNums(velNums(i));                 
            }

            for (int i = 0; i < accNums.size(); i++)
            {
                fixAccsNums(i) = fixPointsNums(accNums(i));                 
            }

            // for (int i = 0; i < path.cols(); i++)
            // {
            //     cout << path.col(i)(0) << "\t" << path.col(i)(1) << "\t"<< path.col(i)(2)<< endl;
            // }
            // cout << fixPointsNums<< endl;
            return;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      Eigen::MatrixX3d &innerPoints,
                                      Eigen::VectorXd &timeAlloc,
                                      Eigen::VectorXd &timeAlloc_,
                                      const Eigen::VectorXi &intp)
        {
            const int pieceN = path.cols() - 1;
            timeAlloc.resize(pieceN);
            timeAlloc_.resize(intp.size());
            innerPoints.resize(pieceN - 1, 3);
            double maxT = 0.0;

            Eigen::Vector3d a, b, c;
            for (int i = 0; i < pieceN; i++)
            {
                a = path.col(i);
                b = path.col(i + 1);
                timeAlloc(i) = 2 * (b - a).norm() / speed;
            }
            int k = 0;
            for (int i = 0; i < intp.size(); i++)
            {
                maxT = timeAlloc.segment(k, intp(i) + 1).maxCoeff();
                for (int j = 0; j < intp(i) + 1; j++)
                {
                   timeAlloc(k + j) = maxT;    
                }
                timeAlloc_(i) = maxT;
                k += intp(i) + 1;
            }
            innerPoints = path.block(0, 1, 3, pieceN - 1).transpose();
            // cout << innerPoints << endl;
        }

        static inline void setInitialVelsAndAccs(Eigen::VectorXd &x, 
                                                 const Eigen::Matrix3Xd &initWayVels, 
                                                 const Eigen::Matrix3Xd &initWayAccs,
                                                 const Eigen::VectorXi &fixVelsNums,
                                                 const Eigen::VectorXi &fixAccsNums, 
                                                 const int &bias, 
                                                 const int &N)
        {
            Eigen::Map<Eigen::MatrixX3d> xiVL(x.data() + bias, N - 1, 3);
            Eigen::Map<Eigen::MatrixX3d> xiAL(x.data() + 2 * bias, N - 1, 3);
            for (int i = 0; i < initWayVels.cols(); i++)
            {
                xiVL.row(fixVelsNums(i)) = initWayVels.col(i).transpose(); 
            }
            for (int i = 0; i < initWayAccs.cols(); i++)
            {
                xiAL.row(fixAccsNums(i)) = initWayAccs.col(i).transpose(); 
            }
        }

        static inline void getAllTimes(Eigen::VectorXd &timeAlloc,
                                       Eigen::VectorXd &timeAlloc_,
                                       const Eigen::VectorXi &intp)
        {
            int k = 0;
            for (int i = 0; i < intp.size(); i++)
            {
                for (int j = 0; j < intp(i) + 1; j++)
                {
                   timeAlloc(k + j) = timeAlloc_(i);    
                }
                k += intp(i) + 1;
            }
        }

    public:
        inline bool setup(const double &timeWeight,
                          const double &energyWeight,
                          const Eigen::Matrix4d  allocMatrix,
                          const Eigen::Matrix4d  allocMatrixI,
                          const Eigen::Matrix<double, 3, 4> &initialPVAL,
                          const Eigen::Matrix<double, 3, 4> &terminalPVAL,
                          const Eigen::Matrix<double, 3, 4> &inifinPVAQ,
                          const Eigen::Matrix3Xd &wayPts,
                          const Eigen::Matrix3Xd &wayVels,
                          const Eigen::Matrix3Xd &wayAccs,
                          const Eigen::VectorXi &velNums,
                          const Eigen::VectorXi &accNums,
                          const int &pieceNum,
                          const int &droneNum,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const double &forceScale,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &angTBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams,
                          const Eigen::Vector3d &continuityDescend,
                          const Eigen::Vector3d &boundaryConditionSlack,
                          const GridMap::Ptr &globalMap)
        {
            rho = timeWeight;
            rhoE = energyWeight;
            droneN = droneNum;
            headPVAL = initialPVAL;
            tailPVAL = terminalPVAL;
            headPVAQ.resize(4, 3 * droneN);
            tailPVAQ.resize(4, 3 * droneN);
            gridMap = globalMap;
            allocM = allocMatrix;
            allocMI = allocMatrixI;

            for(int i = 0; i < droneN; i++)
            {
                headPVAQ.block<4, 3>(0, i * 3) = inifinPVAQ.transpose();
                tailPVAQ.block<4, 3>(0, i * 3) = inifinPVAQ.transpose();
            }

            smoothFactor = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            physicalPm = physicalParams;
            continuityDes = continuityDescend;
            // cout << "qqqqqqq"  << continuityDes.sum() << endl;
            boundaryCondSlack = boundaryConditionSlack;
            // cout << "ppppppp"  << boundaryCondSlack.sum() << endl;
            allocSpeed = magnitudeBd(5);
            disBetweenWaypoints = magnitudeBd(8);
            scale = forceScale;

            angTBs = angTBounds;
            getInitPath(disBetweenWaypoints, headPVAL.col(0), tailPVAL.col(0), wayPts, wayVels, wayAccs, velNums, accNums, initPath, fixPtsNums, fixVelsNums, fixAccsNums, intpNum);
            initWayVels = wayVels;
            initWayAccs = wayAccs;
            inifinAngT = inifinPVAQ.col(0);
            // cout << inifinAngT(0) / M_PI << "\t" << inifinAngT(1)/ M_PI << "\t" << inifinAngT(2)<< endl;
            pieceN = initPath.cols() - 1;

            temporalDim = intpNum.size();
            spatialDimL = 3 * (pieceN - 1);
            spatialDimQ = droneN * 3 * (pieceN - 1);
            continuityDesDim = droneN * continuityDes.sum() * (pieceN - 1);
            boundaryCondSlackDim = droneN * boundaryCondSlack.sum() * 2;
            iter_num_ = 0;

            // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
            minco.setConditions(headPVAL, tailPVAL, headPVAQ, tailPVAQ, continuityDes, boundaryCondSlack, fixPtsNums, fixVelsNums, fixAccsNums, pieceN, droneN, rhoE);
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(6), physicalPm(8), physicalPm(9), physicalPm(10), physicalPm(11));
            
            // allocate memory for optimization

            qs.resize(droneN);
            gs.resize(droneN);
            angT.resize(droneN);
            cst.resize(droneN);
            csp.resize(droneN);
            accQs.resize(droneN);
            rpm1.resize(droneN);
            rpm2.resize(droneN);
            rpm3.resize(droneN);
            rpm4.resize(droneN);

            gradQ.resize(droneN);
            gradF.resize(droneN);
            accs.resize(pieceN * integralRes + 1);
            for (int k = 0; k < droneN; k++)
            {
                accQs[k].resize(pieceN * integralRes + 1);
                rpm1[k].resize(pieceN * integralRes + 1);
                rpm2[k].resize(pieceN * integralRes + 1);
                rpm3[k].resize(pieceN * integralRes + 1);
                rpm4[k].resize(pieceN * integralRes + 1);
            }            

            // Allocate temp variables
            points.resize(pieceN - 1, 3);
            lambdas.resize(droneN * 3 * (pieceN - 1));
            gradByPoints.resize(pieceN - 1, 3);
            gradByVs.resize(pieceN - 1, 3);
            gradByAs.resize(pieceN - 1, 3);
            partialGradByCoeffs.resize((8 * pieceN) * (3 + 3 * droneN));

            isPrint = true;

            return true;
        }

        inline double optimize(std::vector<Trajectory<7>> &trajs, std::vector<Trajectory<7>> &initTrajs, const double &relCostTol)
        {
            int dim = temporalDim + 3 * spatialDimL + 3 * spatialDimQ + continuityDesDim + boundaryCondSlackDim;
            // cout << "dim\t" << dim << endl;
            Eigen::VectorXd x(dim);
            x.setZero();
            Eigen::Map<Eigen::VectorXd> tau(x.data() + 3 * spatialDimL + 3 * spatialDimQ + continuityDesDim + boundaryCondSlackDim, temporalDim);
            setInitial(initPath, allocSpeed, points, times, times_, intpNum);
            backwardT(times_, tau);                              // from init T to tau
            // cout << "adb\t"<<  x(dim - 1) << endl;
            backwardP(points, inifinAngT, x, angTBs, pieceN, droneN); //  from initAngT to lams
            setInitialVelsAndAccs(x, initWayVels, initWayAccs, fixVelsNums, fixAccsNums, spatialDimL + spatialDimQ, pieceN);
            Eigen::Map<const Eigen::VectorXd> xiL(x.data(), spatialDimL);
            Eigen::Map<const Eigen::VectorXd> xiQ(x.data() + spatialDimL, spatialDimQ);
            Eigen::Map<const Eigen::MatrixX3d> xiVL(x.data() + spatialDimL + spatialDimQ, pieceN - 1, 3);
            Eigen::Map<const Eigen::VectorXd> xiVdAngTQ(x.data() + 2 * spatialDimL + spatialDimQ, spatialDimQ);
            Eigen::Map<const Eigen::MatrixX3d> xiAL(x.data() + 2 * spatialDimL + 2 * spatialDimQ, pieceN - 1, 3);
            Eigen::Map<const Eigen::VectorXd> xiAd2AngTQ(x.data() + 3 * spatialDimL + 2 * spatialDimQ, spatialDimQ);
            Eigen::Map<Eigen::VectorXd> xiCD(x.data() + 3 * spatialDimL + 3 * spatialDimQ, continuityDesDim);
            Eigen::Map<Eigen::VectorXd> xiBS(x.data() + 3 * spatialDimL + 3 * spatialDimQ + continuityDesDim, boundaryCondSlackDim);
            // cout << "AAAAAAAAA\t" << xiCD.size() << endl;
            // cout << "BBBBBBBBB\t" << xiBS.size() << endl;
            forwardT(tau, times_);
            forwardP(xiL, xiQ, points, angTs, pieceN, droneN, angTBs); //
            getAllTimes(times, times_, intpNum);
            minco.setParameters(points, angTs, xiVL, xiVdAngTQ, xiAL, xiAd2AngTQ, xiCD, xiBS, times);
            // cout <<"===============================================" << endl;
            // cout << points << endl;
            // cout <<"===============================================" << endl;
            // cout << angTs << endl;
            // cout <<"===============================================" << endl;
            // cout << xiCD << endl;
            // cout <<"===============================================" << endl;
            // cout << xiBS << endl;
            // cout <<"===============================================" << endl;
            // cout << times << endl;
            minco.getTrajs(initTrajs);
            // for (int i = 0; i < 4; i++)
            // {
            //     for (int j = 0; j < initTrajs[i].getPieceNum(); j++)
            //     {
            //         cout << i << "\t" << j << "\t" << initTrajs[i].getPiece(j).getCoeffMat() << endl;
            //     }
            // }
            // cout << initTrajs[0].getPos(1.0 * time) << endl;
            // cout << initTrajs[0].getPos(2.0 * time) << endl;
            // cout << initTrajs[1].getPos(1.5 * time) / M_PI<< endl;
            // cout << initTrajs[2].getPos(2.5 * time) / M_PI<< endl;
            // cout << initTrajs[3].getPos(3.5 * time) / M_PI<< endl;
            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0;
            lbfgs_params.delta = relCostTol;
            lbfgs_params.max_iterations =200000;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_PolytopeSFC::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params,
                                            isPrint);

            if (ret >= 0)
            {
                // cout <<"ret\t" << ret << endl;
                forwardT(tau, times_);
                forwardP(xiL, xiQ, points, angTs, pieceN, droneN, angTBs); //
                minco.setParameters(points, angTs, xiVL, xiVdAngTQ, xiAL, xiAd2AngTQ, xiCD, xiBS, times);
                // cout << "==============================================="<< endl;
                // cout << xiCD<< endl;
                // cout << "==============================================="<< endl;
                // cout << xiBS<<endl;
                // cout << "total time\t" << times.sum()<< endl;
                minco.getTrajs(trajs);
            }
            else
            {
                trajs.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };

}

#endif
