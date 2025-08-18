#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/display.hpp"
#include "gcopter/maneuver.hpp"
#include "plan_env/grid_map.h"
#include "quadrotor_msgs/PolyTraj.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMagL;
    double maxVelMagQ;
    double maxAccMagL;
    double maxBdrMag;
    double maxTiltAngle;
    double maxRPM;
    double maxTheta;
    double maxPsi;
    double minTheta;
    double minPsi;
    double maxT;
    double minT;
    double minThrust;
    double maxThrust;
    double minDistance;
    double clearence;
    double cableLength;
    double massL;
    double massQ;
    double Jxx, Jyy, Jzz;
    double za, MTM;
    double cofthr, coftor;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    double weightE;
    double disBetweenWaypoints;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    int pieceNum;
    int droneNum;
    double percent;
    std::vector<double> inifinAngT;
    std::vector<double> offsets;
    std::vector<double> cofs;
    std::vector<double> rotationalInertia;
    std::vector<double> initLoadPos;
    std::vector<double> endLoadPos;
    std::vector<double> posOffset;
    std::vector<double> continuityDescend;
    std::vector<double> boundaryConditionSlack;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("Percent", percent);
        nh_priv.getParam("MaxVelMagL", maxVelMagL);
        nh_priv.getParam("MaxVelMagQ", maxVelMagQ);
        nh_priv.getParam("MaxAccMagL", maxAccMagL);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MaxRPM", maxRPM);
        nh_priv.getParam("MinTheta", minTheta);
        nh_priv.getParam("MaxTheta", maxTheta);
        nh_priv.getParam("MinPsi", minPsi);
        nh_priv.getParam("MaxPsi", maxPsi);
        nh_priv.getParam("MinT", minT);
        nh_priv.getParam("MaxT", maxT);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("MinDistance", minDistance);
        nh_priv.getParam("CableLength", cableLength);
        nh_priv.getParam("MassL", massL);
        nh_priv.getParam("MassQ", massQ);
        nh_priv.getParam("J", rotationalInertia);
        nh_priv.getParam("Offsets", offsets);
        nh_priv.getParam("Cofs", cofs);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("WeightE", weightE);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("PieceNum", pieceNum);
        nh_priv.getParam("DroneNum", droneNum);
        nh_priv.getParam("InifinAngT", inifinAngT);
        nh_priv.getParam("InitLoadPos", initLoadPos);
        nh_priv.getParam("EndLoadPos", endLoadPos);
        nh_priv.getParam("PosOffset", posOffset);
        nh_priv.getParam("DisBetweenWaypoints", disBetweenWaypoints);
        nh_priv.getParam("ContinuityDescend", continuityDescend);
        nh_priv.getParam("BoundaryConditionSlack", boundaryConditionSlack);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;
    std::vector<ros::Publisher> odom_pubs;

    bool mapInitialized;
    bool isSuccessOpted = false;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    Display display;
    std::vector<Eigen::Vector3d> startGoal;
    Eigen::Matrix3Xd wayPts, wayVels, wayAccs;
    Eigen::VectorXi velNums, accNums;

    std::vector<Trajectory<7>> optTrajs;
    std::vector<Trajectory<7>> initTrajs;
    double trajStamp;
    double scale;
    GridMap::Ptr grid_map_;
    std::vector<ros::Publisher> polyTrajPub;
    std::vector<quadrotor_msgs::PolyTraj> trajMsgs;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        trajMsgs.resize(config.droneNum + 1);
        polyTrajPub.resize(config.droneNum + 1);
        polyTrajPub[0] = nh.advertise<quadrotor_msgs::PolyTraj>("load_planning/trajLoad", 10);
        for (int i = 0; i < config.droneNum; i++)
        {
            polyTrajPub[i + 1] = nh.advertise<quadrotor_msgs::PolyTraj>("drone_" + to_string(i + 1) + "_planning/trajCable", 10);
        }
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        grid_map_.reset(new GridMap);

        grid_map_->initMap();

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            if (true)
            {
                Eigen::Matrix4d aM, aMI;
                Eigen::Matrix<double, 3, 4> iniStateL, finStateL;
                Eigen::Matrix<double, 3, 4> inifinStateQ;
                Eigen::Vector3d angT;
                angT(0) = config.inifinAngT[0] * M_PI;
                angT(1) = config.inifinAngT[1] * M_PI;
                angT(2) = config.inifinAngT[2];
                iniStateL << startGoal.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finStateL << startGoal.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                inifinStateQ << angT, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                Eigen::VectorXd magnitudeBounds(11);
                Eigen::VectorXd angTBounds(6);
                Eigen::VectorXd penaltyWeights(3);
                Eigen::VectorXd physicalParams(12);
                Eigen::Vector3d continuityDescend;
                Eigen::Vector3d boundaryConditionSlack;
                continuityDescend(0) = config.continuityDescend[0];
                continuityDescend(1) = config.continuityDescend[1];
                continuityDescend(2) = config.continuityDescend[2];
                boundaryConditionSlack(0) = config.boundaryConditionSlack[0];
                boundaryConditionSlack(1) = config.boundaryConditionSlack[1];
                boundaryConditionSlack(2) = config.boundaryConditionSlack[2];
                magnitudeBounds(0) = config.maxVelMagQ;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                magnitudeBounds(5) = config.maxVelMagL;
                magnitudeBounds(6) = config.maxAccMagL;
                magnitudeBounds(7) = config.minDistance;
                magnitudeBounds(8) = config.disBetweenWaypoints;
                magnitudeBounds(9) = config.clearence;
                magnitudeBounds(10) = config.maxRPM;
                angTBounds(0) = config.maxTheta * M_PI;
                angTBounds(1) = config.minTheta * M_PI;
                angTBounds(2) = config.maxPsi * M_PI;
                angTBounds(3) = config.minPsi * M_PI;
                angTBounds(4) = config.maxT;
                angTBounds(5) = config.minT;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                physicalParams(0) = config.massQ;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                physicalParams(6) = config.cableLength;
                physicalParams(7) = config.massL;
                physicalParams(8) = config.offsets[0];
                physicalParams(9) = config.rotationalInertia[0];
                physicalParams(10) = config.rotationalInertia[1];
                physicalParams(11) = config.rotationalInertia[2];
                const int quadratureRes = config.integralIntervs;
                scale = physicalParams(1) * physicalParams(7) / (config.droneNum * cos(config.inifinAngT[0] * M_PI));

                Eigen::Quaterniond q1 = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ())); // 绕 Z 轴旋转 90°
                Eigen::Quaterniond q2 = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())); // 绕 Y 轴旋转 90°

                // 四元数乘法
                Eigen::Quaterniond q_product = q1.inverse() * q2; // 先 q2（Y轴），再 q1（Z轴）

                // 验证旋转顺序
                Eigen::Vector3d v = Eigen::Vector3d::UnitX();     // 初始向量 (1, 0, 0)
                Eigen::Vector3d v1, v2;     // 初始向量 (1, 0, 0)
                Eigen::Vector3d v_rotated = q_product * v; // 应用组合旋转
                v1 = q1 * v;
                v2 = q2 * v;     

                aM.row(0) << 1.0, 1.0, 1.0, 1.0;
                aM.row(1) << -1.0, 1.0, 1.0, -1.0;
                aM.row(2) << -1.0, 1.0, -1.0, 1.0;
                aM.row(3) << -1.0, -1.0, 1.0, 1.0;
                aM.row(0) *= config.cofs[0];
                aM.row(1) *= sqrt(2) / 4.0 * config.offsets[1] * config.cofs[0];
                aM.row(2) *= sqrt(2) / 4.0 * config.offsets[1] * config.cofs[0];
                aM.row(3) *= config.cofs[1];
                aMI = aM.inverse();
                // aMI = aM.fullPivLu().inverse();
                // cout << "aM\n"
                //      << aM << endl;
                // cout << "aMI\n"
                //      << aMI << endl;

                optTrajs.clear();
                optTrajs.resize(config.droneNum + 1);
                initTrajs.clear();
                initTrajs.resize(config.droneNum + 1);
                display.reset(physicalParams, angTBounds, magnitudeBounds, config.droneNum, scale, quadratureRes);
                if (!gcopter.setup(config.weightT,
                                   config.weightE,
                                   aM,
                                   aMI,
                                   iniStateL, finStateL,
                                   inifinStateQ,
                                   wayPts,
                                   wayVels,
                                   wayAccs,
                                   velNums,
                                   accNums,
                                   config.pieceNum,
                                   config.droneNum,
                                   INFINITY,
                                   config.smoothingEps,
                                   scale,
                                   quadratureRes,
                                   magnitudeBounds,
                                   angTBounds,
                                   penaltyWeights,
                                   physicalParams,
                                   continuityDescend,
                                   boundaryConditionSlack,
                                   grid_map_))
                    return;
                {
                }

                if (std::isinf(gcopter.optimize(optTrajs, initTrajs, config.relCostTol)))
                {
                    return;
                }

                isSuccessOpted = true;

                polyTraj2ROSMsg(trajMsgs);

                for (int i = 0; i < config.droneNum + 1; i++)
                {
                    polyTrajPub[i].publish(trajMsgs[i]);
                }

                if (initTrajs[0].getPieceNum() > 0 && optTrajs[0].getPieceNum() > 0)
                {
                    // display.displayTrajs(visualizer, initTrajs, config.percent, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), 0.1, 0);
                    display.displayTrajs(visualizer, optTrajs, config.percent, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), 0.1, 1);
                    // display.displayDetails(visualizer, initTrajs, optTrajs, gcopter.getRPM1(), gcopter.getRPM2(), gcopter.getRPM3(), gcopter.getRPM4(), gcopter.getAcc(), gcopter.getAccQs());
                    // display.displayAllows(visualizer, optTrajs, config.percent);
                    // display.displayBounds(visualizer, optTrajs, config.percent);
                    trajStamp = ros::Time::now().toSec();
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        config.initLoadPos[0] += config.posOffset[0];
        config.initLoadPos[1] += config.posOffset[1];
        config.initLoadPos[2] += config.posOffset[2];

        config.endLoadPos[0] += config.posOffset[0];
        config.endLoadPos[1] += config.posOffset[1];
        config.endLoadPos[2] += config.posOffset[2];

        const Eigen::Vector3d start(config.initLoadPos[0], config.initLoadPos[1], config.initLoadPos[2]);
        const Eigen::Vector3d goal(config.endLoadPos[0], config.endLoadPos[1], config.endLoadPos[2]);
        visualizer.visualizeStartGoal(start, 0.1, startGoal.size());
        startGoal.emplace_back(start);
        visualizer.visualizeStartGoal(goal, 0.1, startGoal.size());
        startGoal.emplace_back(goal);

        setInitLoadWayPts();
        cout << setiosflags(ios::fixed) << setprecision(15) << setiosflags(ios::left);
        plan();
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(12);
        physicalParams(0) = config.massQ;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        physicalParams(6) = config.cableLength;
        physicalParams(7) = config.massL;
        physicalParams(8) = config.offsets[0];
        physicalParams(9) = config.rotationalInertia[0];
        physicalParams(10) = config.rotationalInertia[1];
        physicalParams(11) = config.rotationalInertia[2];

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(6), physicalParams(8), physicalParams(9), physicalParams(10), physicalParams(11));
        if (isSuccessOpted)
        {
            // cout <<"nnnnnnnnnnn" << endl;
            odom_pubs.resize(config.droneNum + 1);
            for (int k = 0; k < config.droneNum + 1; k++)
            {
                odom_pubs[k] = nh.advertise<nav_msgs::Odometry>("drone_" + to_string(k) + "_odom", 1);
            }
            const double delta = ros::Time::now().toSec() - trajStamp;

            if (delta > 0.0 && delta < optTrajs[0].getTotalDuration())
            {

                double thr;
                double len = config.cableLength;
                Eigen::Vector4d quat, g, ftau;
                Eigen::Vector3d omg, angT, dAngT, d2AngT, d3AngT, d4AngT, d5AngT, q, dq, d2q, d3q, d4q, d5q, dFs;
                Eigen::Vector3d posL, velL, accL, jerL, snpL, posQ, velQ, accQ, jerQ, snpQ;
                Eigen::VectorXd d2qb, d5qb, d3qb, d4qb;
                Eigen::Vector2d cst, csp;
                std::vector<nav_msgs::Odometry> odoms;
                d2qb.resize(4);
                d3qb.resize(6);
                d4qb.resize(6);
                d5qb.resize(4);
                posL = optTrajs[0].getPos(delta);
                velL = optTrajs[0].getVel(delta);
                accL = optTrajs[0].getAcc(delta);
                jerL = optTrajs[0].getJer(delta);
                snpL = optTrajs[0].getSnp(delta);
                odoms.resize(config.droneNum + 1);
                odoms[0].pose.pose.position.x = posL(0);
                odoms[0].pose.pose.position.y = posL(1);
                odoms[0].pose.pose.position.z = posL(2);
                odoms[0].pose.pose.orientation.w = 1.0;
                odoms[0].pose.pose.orientation.x = 0.0;
                odoms[0].pose.pose.orientation.y = 0.0;
                odoms[0].pose.pose.orientation.z = 0.0;
                odoms[0].twist.twist.linear.x = velL(0);
                odoms[0].twist.twist.linear.y = velL(1);
                odoms[0].twist.twist.linear.z = velL(2);
                odoms[0].twist.twist.angular.x = 0.0;
                odoms[0].twist.twist.angular.y = 0.0;
                odoms[0].twist.twist.angular.z = 0.0;
                for (int k = 0; k < config.droneNum; k++)
                {
                    angT = optTrajs[k + 1].getPos(delta);
                    angT(1) += 2 * k * M_PI / config.droneNum;
                    dAngT = optTrajs[k + 1].getVel(delta);
                    d2AngT = optTrajs[k + 1].getAcc(delta);
                    d3AngT = optTrajs[k + 1].getJer(delta);
                    d4AngT = optTrajs[k + 1].getSnp(delta);
                    cst = getcs(angT(0));
                    csp = getcs(angT(1));
                    // g = getG(cst, csp);
                    q = getQ(cst, csp, g);
                    getd5Qs(dAngT.head(2), d2AngT.head(2), d3AngT.head(2), d4AngT.head(2), d5AngT.head(2), cst, csp, g, d2qb, d3qb, d4qb, d5qb, dq, d2q, d3q, d4q, d5q);
                    posQ = posL + len * q;
                    velQ = velL + len * dq;
                    accQ = accL + len * d2q;
                    jerQ = jerL + len * d3q;
                    snpQ = snpL + len * d4q;
                    dFs << scale * angT(2), scale * dAngT(2), scale * d2AngT(2);
                    flatmap.Forward(accQ, jerQ, snpQ, q, dq, d2q, dFs, 0.0, 0.0, 0.0, ftau, thr, quat, omg);
                    odoms[k + 1].pose.pose.position.x = posQ(0);
                    odoms[k + 1].pose.pose.position.y = posQ(1);
                    odoms[k + 1].pose.pose.position.z = posQ(2);
                    odoms[k + 1].pose.pose.orientation.w = quat(0);
                    odoms[k + 1].pose.pose.orientation.x = quat(1);
                    odoms[k + 1].pose.pose.orientation.y = quat(2);
                    odoms[k + 1].pose.pose.orientation.z = quat(3);
                    odoms[k + 1].twist.twist.linear.x = velQ(0);
                    odoms[k + 1].twist.twist.linear.y = velQ(1);
                    odoms[k + 1].twist.twist.linear.z = velQ(2);
                    odoms[k + 1].twist.twist.angular.x = omg(0);
                    odoms[k + 1].twist.twist.angular.y = omg(1);
                    odoms[k + 1].twist.twist.angular.z = omg(2);
                }
                for (int k = 0; k < config.droneNum + 1; k++)
                {
                    odom_pubs[k].publish(odoms[k]);
                }
            }
        }
    }

    inline void setInitLoadWayPts()
    {
        double init_x, init_y, init_z;
        double vel, acc;
        int velCols, accCols;
        Eigen::Vector3d vec;
        Eigen::Vector3d posOffset;
        init_x = config.initLoadPos[0];
        init_y = config.initLoadPos[1];
        init_z = config.initLoadPos[2];

        posOffset(0) = config.posOffset[0];
        posOffset(1) = config.posOffset[1];
        posOffset(2) = config.posOffset[2];
        traj_small_circle(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums, posOffset, init_z);

    }

    void polyTraj2ROSMsg(std::vector<quadrotor_msgs::PolyTraj> &msgs)
    {
        for (int k = 0; k < config.droneNum + 1; k++)
        {
            auto data = &optTrajs[k];

            msgs[k].drone_id = k;
            msgs[k].order = 7; // todo, only support order = 5 now.
            if (k == 0)
            {
                msgs[k].type = 0;
            }
            else
            {
                msgs[k].type = 1;
            }

            Eigen::VectorXd durs = data->getDurations();
            int piece_num = data->getPieceNum();
            msgs[k].duration.resize(piece_num);
            msgs[k].coef_x.resize(8 * piece_num);
            msgs[k].coef_y.resize(8 * piece_num);
            msgs[k].coef_z.resize(8 * piece_num);
            for (int i = 0; i < piece_num; ++i)
            {
                msgs[k].duration[i] = durs(i);

                Piece<7>::CoefficientMat cMat = data->getPiece(i).getCoeffMat();
                int i8 = i * 8;
                for (int j = 0; j < 8; j++)
                {
                    msgs[k].coef_x[i8 + j] = cMat(0, j);
                    msgs[k].coef_y[i8 + j] = cMat(1, j);
                    msgs[k].coef_z[i8 + j] = cMat(2, j);
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
