#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/funs.hpp"

#include <fstream>
#include <vector>

using namespace std;
using namespace gcopter;

class Display // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
{
private:
    int dN, pN, wN, n;
    int integralRes;
    std::vector<std::vector<Eigen::Vector3d>> angTs, posQs, qs, upperT, lowerT, upperRPM, lowerRPM, upperTheta, lowerTheta, upperPsi, lowerPsi, theta, psi, T, thetaWpt, psiWpt, TWpt, errWpt, errs, 
                                              rpm1Max, rpm1Min, rpm2Max, rpm2Min, rpm3Max, rpm3Min, rpm4Max, rpm4Min, rpm1, rpm2, rpm3, rpm4, accl, accQ;
    std::vector<std::vector<Eigen::Vector2d>> csts, csps;
    std::vector<Eigen::Vector3d> posLs, arrows, bs, Pts;

    std::vector<Eigen::Vector3d> accs, forces;
    Eigen::Vector3d selPosQ, angT, acc, vel, q, force;
    Eigen::VectorXd offset;
    Eigen::Vector4d g;
    double dt = 0.01, dT, t1;
    double scl, ratio;
    double err;
    double bias;
    Eigen::VectorXd phyParams, angTBds, magnBds;

public:
    inline void reset(const Eigen::VectorXd &physicalParams, const Eigen::VectorXd &angTBounds, const Eigen::VectorXd &magnitudeBounds, 
                      const int &droneN, const double &scale, const int & integralResolution)
    {
        phyParams = physicalParams;
        angTBds = angTBounds;
        magnBds = magnitudeBounds;
        dN = droneN;
        scl = scale;
        t1 = 10.0;
        bias = 15.0;
        integralRes = integralResolution;
        g.setZero();
        offset.resize(10);
        return;
    }
    inline void displayTrajs(Visualizer &visualizer, const std::vector<Trajectory<7>> &trajs, const double &per, Eigen::Vector4d color, const double &scale, int id)
    {
        // cout << "ACFT" << endl;
        pN = floor(trajs[0].getTotalDuration() / dt);
        n = floor(trajs[0].getTotalDuration() * per * 0.01 / dt);
        posLs.resize(pN + 1);
        posQs.resize(dN);
        qs.resize(dN);
        angTs.resize(dN);
        csts.resize(dN);
        csps.resize(dN);
        Eigen::Vector4d colorL;
        colorL << 0.0, 0.5, 1.0, 1.0;
        for (int i = 0; i <= pN; i++)
        {
            posLs[i] = trajs[0].getPos(i * dt);
            if (id == 1)
            {
                // cout << "pos\t"<< i << "\t" << posLs[i](0) << "\t" << posLs[i](1) << "\t" << posLs[i](2) << "\t" << trajs[0].getJer(i * dt).norm() << endl;
            }
        }
        visualizer.displayMarkerList(posLs, 0.2, colorL, 1000 + id);
        for (int k = 0; k < dN; k++)
        {
            posQs[k].resize(pN + 1);
            qs[k].resize(pN + 1);
            angTs[k].resize(pN + 1);
            csts[k].resize(pN + 1);
            csps[k].resize(pN + 1);
            for (int i = 0; i <= pN; i++)
            {
                angTs[k][i] = trajs[k + 1].getPos(i * dt);
                // cout << k << "\t"<< i << "\t" << angTs[k][i](0) / M_PI << "\t" << angTs[k][i](1) / M_PI << "\t" << angTs[k][i](2)<< endl;
                csts[k][i] = getcs(angTs[k][i](0));
                csps[k][i] = getcs(angTs[k][i](1) + 2 * M_PI * k / dN);
                qs[k][i] = getQ(csts[k][i], csps[k][i], g);
                posQs[k][i] = posLs[i] + phyParams(6) * qs[k][i];
                // cout << k << "\t"<< i << "\t" << posQs[k][i](0) << "\t" << posQs[k][i](1) << "\t" << posQs[k][i](2)<< endl;
            }
            visualizer.displayMarkerList(posQs[k], 0.1, color, 1000 + 2 + dN * id + k);
            selPosQ = posLs[n] + phyParams(6) * qs[k][n];
            visualizer.displaySelectedPos(selPosQ, color, 0.2, 1000 + dN * id + k);
        }
        return ;
    }

    inline void displayDetails(Visualizer &visualizer, const std::vector<Trajectory<7>> &initTrajs, const std::vector<Trajectory<7>> &optTrajs,
                               const std::vector<Eigen::VectorXd> rpm1s, const std::vector<Eigen::VectorXd> rpm2s, const std::vector<Eigen::VectorXd> rpm3s,
                               const std::vector<Eigen::VectorXd> rpm4s,
                               const Eigen::VectorXd& accL, const std::vector<Eigen::VectorXd> accQs)
    {
        // cout << "AAAAAAAA\t" << optTrajs[0].getDurations() << endl;
        ratio = initTrajs[0].getTotalDuration() / optTrajs[0].getTotalDuration();
        accs.resize(pN + 1);
        forces.resize(pN + 1);
        upperTheta.resize(dN);
        upperPsi.resize(dN);
        upperT.resize(dN);
        lowerTheta.resize(dN);
        lowerPsi.resize(dN);
        lowerT.resize(dN);
        theta.resize(dN);
        psi.resize(dN);
        T.resize(dN);
        thetaWpt.resize(dN);
        psiWpt.resize(dN);
        TWpt.resize(dN);
        errs.resize(dN);
        errWpt.resize(dN);

        accQ.resize(dN);
        rpm1Max.resize(dN);
        rpm1Min.resize(dN);
        rpm2Max.resize(dN);
        rpm2Min.resize(dN);
        rpm3Max.resize(dN);
        rpm3Min.resize(dN);
        rpm4Max.resize(dN);
        rpm4Min.resize(dN);
        rpm1.resize(dN);
        rpm2.resize(dN);
        rpm3.resize(dN);
        rpm4.resize(dN);
        accl.resize(dN);
        wN = optTrajs[0].getPieceNum();
        Pts.resize(wN -1);

        for (int i = 0; i < wN - 1; i++)
        {
            Pts[i] = optTrajs[0].getJuncPos(i);
        }
        visualizer.displayMarkerList(Pts, 0.3, Eigen::Vector4d(0.0, 0.5, 1.0, 1.0), 2800000, true, false);

        for (int k = 0; k < dN; k++)
        {
            upperTheta[k].resize(pN + 1);
            lowerTheta[k].resize(pN + 1);
            theta[k].resize(pN + 1);

            upperPsi[k].resize(pN + 1);
            lowerPsi[k].resize(pN + 1);
            psi[k].resize(pN + 1);

            upperT[k].resize(pN + 1);
            lowerT[k].resize(pN + 1);
            T[k].resize(pN + 1);

            errs[k].resize(pN + 1);
            thetaWpt[k].resize(wN - 1);
            psiWpt[k].resize(wN - 1);
            TWpt[k].resize(wN - 1);
            errWpt[k].resize(wN - 1);
            rpm1Max[k].resize(wN * integralRes + 1);
            rpm1Min[k].resize(wN * integralRes + 1);
            rpm2Max[k].resize(wN * integralRes + 1);
            rpm2Min[k].resize(wN * integralRes + 1);
            rpm3Max[k].resize(wN * integralRes + 1);
            rpm3Min[k].resize(wN * integralRes + 1);
            rpm4Max[k].resize(wN * integralRes + 1);
            rpm4Min[k].resize(wN * integralRes + 1);
            rpm1[k].resize(wN * integralRes + 1);
            rpm2[k].resize(wN * integralRes + 1);
            rpm3[k].resize(wN * integralRes + 1);
            rpm4[k].resize(wN * integralRes + 1);
            accl[k].resize(wN * integralRes + 1);
            accQ[k].resize(wN * integralRes + 1);
        }
        for (int i = 0; i <= pN; i++)
        {
            accs[i] = optTrajs[0].getAcc(i * dt);
            accs[i](2) += phyParams(1);
            accs[i] *= phyParams(7);
            forces[i].setZero();
            for (int k = 0; k < dN; k++)
            {
                forces[i] += scl * angTs[k][i](2) * qs[k][i];
            }
        }

        offset << -4.0, -6.0, -10.0, -12.0, -14.0,  -17.0, -20.0, -23.0, -26.0, -36.0;     // 1,theta,  2,psi,  3,T,  4,force error,  5,vel,  6,omg, 7,tilt angle, 8,thrust   
        for (int k = 0; k < dN; k++)
        {
            for (int i = 0; i <= pN; i++)
            {
                upperTheta[k][i](0) = i * dt * ratio + k * bias;
                upperTheta[k][i](1) = angTBds(0) + offset(0);
                upperTheta[k][i](2) = 0.0;
                lowerTheta[k][i](0) = i * dt * ratio + k * bias;
                lowerTheta[k][i](1) = angTBds(1) + offset(0);
                lowerTheta[k][i](2) = 0.0;
                theta[k][i](0) = i * dt * ratio + k * bias;
                theta[k][i](1) = angTs[k][i](0) + offset(0);
                theta[k][i](2) = 0.0;

                upperPsi[k][i](0) = i * dt * ratio + k * bias;
                upperPsi[k][i](1) = angTBds(2) + offset(1);
                upperPsi[k][i](2) = 0.0;
                lowerPsi[k][i](0) = i * dt * ratio + k * bias;
                lowerPsi[k][i](1) = angTBds(3) + offset(1);
                lowerPsi[k][i](2) = 0.0;
                psi[k][i](0) = i * dt * ratio + k * bias;
                psi[k][i](1) = angTs[k][i](1) + offset(1);
                psi[k][i](2) = 0.0;

                upperT[k][i](0) = i * dt * ratio + k * bias;
                upperT[k][i](1) = angTBds(4) + offset(2);
                upperT[k][i](2) = 0.0;
                lowerT[k][i](0) = i * dt * ratio + k * bias;
                lowerT[k][i](1) = angTBds(5) + offset(2);
                // lowerT[k][i](1) = (180.0 * acos(qs[k][i](2)) / M_PI - 30.0) * 2.5 + offset(2);
                lowerT[k][i](2) = 0.0;
                T[k][i](0) = i * dt * ratio + k * bias;
                T[k][i](1) = angTs[k][i](2) + offset(2);
                T[k][i](2) = 0.0;

                errs[k][i](0) = i * dt * ratio + k * bias;
                errs[k][i](1) = sqrt((forces[i] - accs[i]).squaredNorm()) + offset(3);
                errs[k][i](2) = 0.0;
            }
            dT = 0.0;
            for (int i = 0; i < wN - 1; i++)
            {
                dT += optTrajs[0].getDurations()(i);

                TWpt[k][i](0) = dT * ratio + k * bias;
                TWpt[k][i](1) = angT(2) + offset(2);
                TWpt[k][i](2) = 0.0;
            }
            visualizer.displayMarkerList(upperTheta[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 0);
            visualizer.displayMarkerList(lowerTheta[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 1);
            visualizer.displayMarkerList(theta[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 2); 
            visualizer.displayMarkerList(thetaWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 3, true, false);
            visualizer.displayMarkerList(upperPsi[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 4);
            visualizer.displayMarkerList(lowerPsi[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 5);
            visualizer.displayMarkerList(psi[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 6); 
            visualizer.displayMarkerList(psiWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 7, true, false);
            visualizer.displayMarkerList(upperT[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 8);
            visualizer.displayMarkerList(lowerT[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 9);
            visualizer.displayMarkerList(T[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 10); 
            visualizer.displayMarkerList(TWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 11, true, false);
            visualizer.displayMarkerList(errs[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 12); 
        }

        std::ofstream p0;

        // for (int k = 0; k < dN; k++)
        // {
        //     p0.open("/home/ros/First_data" + std::to_string(k) + ".csv",ios::out|ios::trunc);
        //     for (int i = 0; i <= pN; i++)
        //     {
        //         p0   << i << ","
        //             << upperTheta[k][i](0) << "," << upperTheta[k][i](1) << "," << upperTheta[k][i](2) << ","
        //             << lowerTheta[k][i](0) << "," << lowerTheta[k][i](1) << "," << lowerTheta[k][i](2) << ","
        //             << theta[k][i](1) << "," << theta[k][i](1) << "," << theta[k][i](2) << ","
        //             << thetaWpt[k][i](0) << "," << thetaWpt[k][i](1) << "," << thetaWpt[k][i](2) << ","
        //             << upperPsi[k][i](0) << "," << upperPsi[k][i](1) << "," << upperPsi[k][i](2) << ","
        //             << lowerPsi[k][i](1) << "," << lowerPsi[k][i](1) << "," << lowerPsi[k][i](2) << ","
        //             << psi[k][i](0) << "," << psi[k][i](1) << "," << psi[k][i](2) << ","
        //             << psiWpt[k][i](0) << "," << psiWpt[k][i](1) << "," << psiWpt[k][i](2) << ","
        //             << upperT[k][i](1) << "," << upperT[k][i](1) << "," << upperT[k][i](2) << ","
        //             << lowerT[k][i](0) << "," << lowerT[k][i](1) << "," << lowerT[k][i](2) << ","
        //             << T[k][i](0) << "," << T[k][i](1) << "," << T[k][i](2) << ","
        //             // << TWpt[k][i](1) << "," << TWpt[k][i](1) << "," << TWpt[k][i](2) << ","
        //             << errs[k][i](1) << "," << errs[k][i](1) << "," << errs[k][i](2) << ",";
        //     }

        //     p0.close();
        // }


        for (int k = 0; k < dN; k++)
        {
            p0.open("/home/ros/First_data" + std::to_string(k) + ".csv",ios::out|ios::trunc);
            for (int i = 0; i <= pN; i++)
            {
                p0   << i << ","
                    << upperTheta[k][i](1) << ","
                    << lowerTheta[k][i](1) << ","
                    << theta[k][i](1) << "," 
                    << upperPsi[k][i](1) << ","
                    << lowerPsi[k][i](1) << ","
                    << psi[k][i](1) << "," 
                    << upperT[k][i](1) << ","
                    << lowerT[k][i](1) << ","
                    << T[k][i](1) << ","
                    << TWpt[k][i](1) << ","
                    << errs[k][i](1) << "\n";
            }

            p0.close();
        }



        dT = 0.0;
        for (int i = 0; i < wN - 1; i++)
        {
            dT += optTrajs[0].getDurations()(i);
            vel = optTrajs[0].getVel(dT);
            acc = optTrajs[0].getAcc(dT);
            // cout << "vel\t" << i  << "\t" << vel(0) << "\t" << vel(1) << "\t" << vel(2) << endl;
            // cout << "acc\t" << i  << "\t" << acc(0) << "\t" << acc(1) << "\t" << acc(2) << endl;
            acc(2) += phyParams(1);
            acc *= phyParams(7);
            force.setZero();
            for (int k = 0; k < dN; k++)
            {
                angT = optTrajs[k + 1].getPos(dT);
                force += scl * angT(2) * getQ(getcs(angT(0)), getcs(angT(1) + 2 * M_PI * k / dN), g);
            }
            err = sqrt((force - acc).squaredNorm());
            for (int k = 0; k < dN; k++)
            {
                errWpt[k][i](0) = dT * ratio + k * bias;
                errWpt[k][i](1) = err + offset(3);
                errWpt[k][i](2) = 0.0;
            }
        }
        for (int k = 0; k < dN; k++)
        {
            visualizer.displayMarkerList(errWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 13, true, false);
        }

        dT = initTrajs[0].getDurations()[0] / integralRes;
        for (int i = 0; i <= wN * integralRes; i++)
        {
            for (int k = 0; k < dN; k++)
            {
                accl[k][i](0) = i * dT + k * bias;
                accl[k][i](1) = accL(i) + offset(4);
                // cout << "vel\t" << k << "\t" << i << "\t" << velQs[k][i] << endl;
                accl[k][i](2) = 0.0;

                rpm1Max[k][i](0) = i * dT + k * bias;
                rpm1Max[k][i](1) = magnBds(10) + offset(5);
                rpm1Max[k][i](2) = 0.0;
                rpm1Min[k][i](0) = i * dT + k * bias;
                rpm1Min[k][i](1) = 0.0 + offset(5);
                rpm1Min[k][i](2) = 0.0;
                rpm1[k][i](0) = i * dT + k * bias;
                rpm1[k][i](1) = rpm1s[k][i] + offset(5);
                // cout << "vel\t" << k << "\t" << i << "\t" << velQs[k][i] << endl;
                rpm1[k][i](2) = 0.0;

                rpm2Max[k][i](0) = i * dT + k * bias;
                rpm2Max[k][i](1) = magnBds(10) + offset(6);
                rpm2Max[k][i](2) = 0.0;
                rpm2Min[k][i](0) = i * dT + k * bias;
                rpm2Min[k][i](1) = 0.0 + offset(6);
                rpm2Min[k][i](2) = 0.0;
                rpm2[k][i](0) = i * dT + k * bias;
                rpm2[k][i](1) = rpm2s[k][i] + offset(6);
                rpm2[k][i](2) = 0.0;
                // cout << "omg\t" << k << "\t" << i << "\t" << omgQs[k][i] << endl;
                rpm3Max[k][i](0) = i * dT + k * bias;
                rpm3Max[k][i](1) = magnBds(10) + offset(7);
                rpm3Max[k][i](2) = 0.0;
                rpm3Min[k][i](0) = i * dT + k * bias;
                rpm3Min[k][i](1) = 0.0 + offset(7);
                rpm3Min[k][i](2) = 0.0;
                rpm3[k][i](0) = i * dT + k * bias;
                rpm3[k][i](1) = rpm3s[k][i] + offset(7);
                rpm3[k][i](2) = 0.0;
                // cout << "att\t" << k << "\t" << i << "\t" << attQs[k][i] * 180.0 / M_PI << endl;
                rpm4Max[k][i](0) = i * dT + k * bias;
                rpm4Max[k][i](1) = magnBds(10) + offset(8);
                rpm4Max[k][i](2) = 0.0;
                rpm4Min[k][i](0) = i * dT + k * bias;
                rpm4Min[k][i](1) = 0.0 + offset(8);
                rpm4Min[k][i](2) = 0.0;
                rpm4[k][i](0) = i * dT + k * bias;
                rpm4[k][i](1) = rpm4s[k][i] + offset(8);
                rpm4[k][i](2) = 0.0;

                accQ[k][i](0) = i * dT + k * bias;
                accQ[k][i](1) = accQs[k][i] + offset(9);
                // cout << "vel\t" << k << "\t" << i << "\t" << velQs[k][i] << endl;
                accQ[k][i](2) = 0.0;
                // cout << "T\t" << k << "\t" << i << "\t" << TQs[k][i] <s< endl;
            }
        }



        std::ofstream p;

        // for (int k = 0; k < dN; k++)
        // {
        //     p.open("/home/ros/Sec_data" + std::to_string(k) + ".csv",ios::out|ios::trunc);
        //     for (int i = 0; i <= wN * integralRes; i++)
        //     {
        //         p   << i << ","
        //             << rpm1Max[k][i](0) << "," << rpm1Max[k][i](1) << "," << rpm1Max[k][i](2) << ","
        //             << rpm1Min[k][i](0) << "," << rpm1Min[k][i](1) << "," << rpm1Min[k][i](2) << ","
        //             << rpm1[k][i](1) << "," << rpm1[k][i](1) << "," << rpm1[k][i](2) << ","
        //             << rpm2Max[k][i](0) << "," << rpm2Max[k][i](1) << "," << rpm2Max[k][i](2) << ","
        //             << rpm2Min[k][i](0) << "," << rpm2Min[k][i](1) << "," << rpm2Min[k][i](2) << ","
        //             << rpm2[k][i](1) << "," << rpm2[k][i](1) << "," << rpm2[k][i](2) << ","
        //             << rpm3Max[k][i](0) << "," << rpm3Max[k][i](1) << "," << rpm3Max[k][i](2) << ","
        //             << rpm3Min[k][i](0) << "," << rpm3Min[k][i](1) << "," << rpm3Min[k][i](2) << ","
        //             << rpm3[k][i](1) << "," << rpm3[k][i](1) << "," << rpm3[k][i](2) << ","
        //             << rpm4Max[k][i](0) << "," << rpm4Max[k][i](1) << "," << rpm4Max[k][i](2) << ","
        //             << rpm4Min[k][i](0) << "," << rpm4Min[k][i](1) << "," << rpm4Min[k][i](2) << ","
        //             << rpm4[k][i](1) << "," << rpm4[k][i](1) << "," << rpm4[k][i](2) << ","
        //             << accQ[k][i](1) << "," << accQ[k][i](1) << "," << accQ[k][i](2) << ","
        //             << accl[k][i](1) << "," << accl[k][i](1) << "," << accl[k][i](2) << "\n";
        //     }

        //     p.close();
        // }


        for (int k = 0; k < dN; k++)
        {
            p.open("/home/ros/Sec_data" + std::to_string(k) + ".csv",ios::out|ios::trunc);
            for (int i = 0; i <= wN * integralRes; i++)
            {
                p   << i << ","
                    << rpm1Max[k][i](1) << "," << rpm1Min[k][i](1) << "," 
                    << rpm1[k][i](1) << ","
                    << rpm2Max[k][i](1) << "," << rpm2Min[k][i](1) << "," 
                    << rpm2[k][i](1) << ","
                    << rpm3Max[k][i](1) << "," << rpm3Min[k][i](1) << "," 
                    << rpm3[k][i](1) << ","
                    << rpm4Max[k][i](1) << "," << rpm4Min[k][i](1) << "," 
                    << rpm4[k][i](1) << ","
                    << accQ[k][i](1) << ","
                    << accl[k][i](1) << "\n";
            }

            p.close();
        }


        for (int k = 0; k < dN; k++)
        {
            visualizer.displayMarkerList(rpm1Max[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 14);
            visualizer.displayMarkerList(rpm1Min[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 15);
            visualizer.displayMarkerList(rpm1[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 16);
            // visualizer.displayMarkerList(accQ[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), (k + 1) * 3000 + 29);
            visualizer.displayMarkerList(rpm2Max[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 17);
            visualizer.displayMarkerList(rpm2Min[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 18);
            visualizer.displayMarkerList(rpm2[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 19);
            visualizer.displayMarkerList(rpm3Max[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 20);
            visualizer.displayMarkerList(rpm3Min[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 21);
            visualizer.displayMarkerList(rpm3[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 22);
            visualizer.displayMarkerList(rpm4Max[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 23);
            visualizer.displayMarkerList(rpm4Min[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 24);
            visualizer.displayMarkerList(rpm4[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 25);
            visualizer.displayMarkerList(accl[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), (k + 1) * 3000 + 29);
            visualizer.displayMarkerList(accQ[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), (k + 1) * 5000 + 100);
            
        }



        return;
    }

    inline void displayAllows(Visualizer &visualizer, const std::vector<Trajectory<7>> &optTrajs, const double &per)
    {
        arrows.resize(2);
        n = floor(optTrajs[0].getTotalDuration() * per * 0.01 / dt);
        arrows[0] = posLs[n];
        arrows[1] = posLs[n] + accs[n] / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(1.0, 0.0, 1.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 0);
        arrows[1] = posLs[n] + forces[n] / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 1);
        arrows[1] = posLs[n] + Eigen::Vector3d(0.0, 0.0, phyParams(1)) / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 2);
        return ;
    }

    inline void displayBounds(Visualizer &visualizer, const std::vector<Trajectory<7>> &optTrajs, const double &per)
    {
        n = floor(optTrajs[0].getTotalDuration() * per * 0.01 / dt);
        arrows[0] = posLs[n];
        int m = 100;
        double psi, dp;
        dp = 2 * M_PI / m;
        bs.resize(m + 1);
        double ctma = cos(angTBds(0));
        double stma = sin(angTBds(0));
        for (int i = 0; i < m + 1; i++)
        {
          psi = i * dp;
          bs[i](0) = posLs[n](0) + phyParams(6) * stma * cos(psi);
          bs[i](1) = posLs[n](1) + phyParams(6) * stma * sin(psi);
          bs[i](2) = posLs[n](2) + 0.0;
        }
        visualizer.displayMarkerList(bs, 0.15, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 26, false, true);
        m = 10;
        for (int k = 0; k < dN; k++)
        {
          psi = 2 * M_PI * k / dN + angTBds(2);
          bs.resize(m + 1);
          for (int i = 0; i < m + 1; i++)
          {
                bs[i](0) = posLs[n](0) + (i * phyParams(6) / m) * stma * cos(psi);
                bs[i](1) = posLs[n](1) + (i * phyParams(6) / m) * stma * sin(psi);
                bs[i](2) = posLs[n](2) + 0.0;
          }
          visualizer.displayMarkerList(bs, 0.15, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 27 + k, false, true);
        }
        return ;
    }
};
