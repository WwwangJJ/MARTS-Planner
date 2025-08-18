#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    inline void getDesOmgs(const Eigen::Vector3d &acc,
                           const Eigen::Vector3d &jer,
                           const Eigen::Vector3d &snp,
                           const Eigen::Vector3d &q,
                           const Eigen::Vector3d &dq,
                           const Eigen::Vector3d &d2q,
                           const double &F,
                           const double &dF,
                           const double &d2F,
                           const double &psi,
                           const double &dpsi,
                           const double &ddpsi,
                           Eigen::Vector4d &quat,
                           Eigen::Vector3d &omg,
                           Eigen::Vector3d &domg,
                           const double &massQ,
                           const double &grav)
    {
        double dFq0, dFq1, dFq2, dzu_norm, zu_sqr_norm, zu_norm;
        double d2Fq0, d2Fq1, d2Fq2;
        double c_half_psi, s_half_psi, c_psi, s_psi;
        double dng00, dng01, dng02, dng11, dng12, dng22, ng_den, ng_den4;
        double dz_term0, dz_term1, dz_term2, d2z_term0, d2z_term1, d2z_term2;
        double z0, z1, z2, dz0, dz1, dz2, d2z0, d2z1, d2z2, zu0, zu1, zu2, zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22;
        double omg_den, omg_term;
        double tilt0, tilt1, tilt2, tilt_den;
        double massFrac = 1.0 / massQ;

        zu0 = acc(0) + massFrac * F * q(0);
        zu1 = acc(1) + massFrac * F * q(1);
        zu2 = acc(2) + massFrac * F * q(2) + grav;
        zu_sqr0 = zu0 * zu0;
        zu_sqr1 = zu1 * zu1;
        zu_sqr2 = zu2 * zu2;
        zu01 = zu0 * zu1;
        zu12 = zu1 * zu2;
        zu02 = zu0 * zu2;
        zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
        zu_norm = sqrt(zu_sqr_norm);
        z0 = zu0 / zu_norm;
        z1 = zu1 / zu_norm;
        z2 = zu2 / zu_norm;
        ng_den = zu_sqr_norm * zu_norm;
        ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
        ng01 = -zu01 / ng_den;
        ng02 = -zu02 / ng_den;
        ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
        ng12 = -zu12 / ng_den;
        ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
        dFq0 = massFrac * (F * dq(0) + dF * q(0));
        dFq1 = massFrac * (F * dq(1) + dF * q(1));
        dFq2 = massFrac * (F * dq(2) + dF * q(2));
        dz_term0 = jer(0) + dFq0;
        dz_term1 = jer(1) + dFq1;
        dz_term2 = jer(2) + dFq2;
        dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
        dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
        dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
        // for domg
        ng_den4 = zu_sqr_norm * zu_sqr_norm;
        dzu_norm = (zu0 * dz_term0 + zu1 * dz_term1 + zu2 * dz_term2) / zu_norm;
        dng00 = (2 * zu_norm * (zu1 * dz_term1 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr1 + zu_sqr2)) / ng_den4;
        dng01 = -((dz_term0 * zu1 + dz_term1 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu1) / ng_den4;
        dng02 = -((dz_term0 * zu2 + dz_term2 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu2) / ng_den4;
        dng11 = (2 * zu_norm * (zu0 * dz_term0 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr0 + zu_sqr2)) / ng_den4;
        dng12 = -((dz_term1 * zu2 + dz_term2 * zu1) * zu_norm - 3 * dzu_norm * zu1 * zu2) / ng_den4;
        dng22 = (2 * zu_norm * (zu0 * dz_term0 + zu1 * dz_term1) - 3 * dzu_norm * (zu_sqr0 + zu_sqr1)) / ng_den4;

        d2Fq0 = massFrac * (F * d2q(0) + 2 * dF * dq(0) + d2F * q(0));
        d2Fq1 = massFrac * (F * d2q(1) + 2 * dF * dq(1) + d2F * q(1));
        d2Fq2 = massFrac * (F * d2q(2) + 2 * dF * dq(2) + d2F * q(2));

        d2z_term0 = snp(0) + d2Fq0;
        d2z_term1 = snp(1) + d2Fq1;
        d2z_term2 = snp(2) + d2Fq2;
        d2z0 = dng00 * dz_term0 + dng01 * dz_term1 + dng02 * dz_term2 + ng00 * d2z_term0 + ng01 * d2z_term1 + ng02 * d2z_term2;
        d2z1 = dng01 * dz_term0 + dng11 * dz_term1 + dng12 * dz_term2 + ng01 * d2z_term0 + ng11 * d2z_term1 + ng12 * d2z_term2;
        d2z2 = dng02 * dz_term0 + dng12 * dz_term1 + dng22 * dz_term2 + ng02 * d2z_term0 + ng12 * d2z_term1 + ng22 * d2z_term2;

        tilt_den = sqrt(2.0 * (1.0 + z2));
        tilt0 = 0.5 * tilt_den;
        tilt1 = -z1 / tilt_den;
        tilt2 = z0 / tilt_den;
        c_half_psi = cos(0.5 * psi);
        s_half_psi = sin(0.5 * psi);
        quat(0) = tilt0 * c_half_psi;
        quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
        quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
        quat(3) = tilt0 * s_half_psi;
        c_psi = cos(psi);
        s_psi = sin(psi);
        omg_den = z2 + 1.0;
        omg_term = dz2 / omg_den;
        omg(0) = dz0 * s_psi - dz1 * c_psi -
                 (z0 * s_psi - z1 * c_psi) * omg_term;
        omg(1) = dz0 * c_psi + dz1 * s_psi -
                 (z0 * c_psi + z1 * s_psi) * omg_term;
        omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;
        domg(0) = s_psi * d2z0 - c_psi * d2z1 - (s_psi * z0 - c_psi * z1) * d2z2 / omg_den + c_psi * dz0 * dpsi + s_psi * dz1 * dpsi - dz2 * (c_psi * z0 * dpsi + s_psi * z1 * dpsi + s_psi * dz0 - c_psi * dz1) / omg_den + dz2 * dz2 * (s_psi * z0 - c_psi * z1) / pow(omg_den, 2);
        domg(1) = c_psi * d2z0 + s_psi * d2z1 - (c_psi * z0 + s_psi * z1) * d2z2 / omg_den + c_psi * dz1 * dpsi - s_psi * dz0 * dpsi - dz2 * (-s_psi * z0 * dpsi + c_psi * z1 * dpsi + c_psi * dz0 + s_psi * dz1) / omg_den + dz2 * dz2 * (c_psi * z0 + s_psi * z1) / pow(omg_den, 2);
        domg(2) = (z1 * d2z0 - z0 * d2z1 - dz1 * dz0 + dz0 * dz1) / omg_den - dz2 * (-dz1 * z0 + dz0 * z1) / pow(omg_den, 2) + ddpsi;

        return;
    }

    class FlatnessMap // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
    {
    public:
        inline void reset(const double &vehicle_mass,
                          const double &gravitational_acceleration,
                          const double &cable_length,
                          const double &offsetZ,
                          const double &Jxx,
                          const double &Jyy,
                          const double &Jzz)
        {
            mass = vehicle_mass;
            massFrac = 1.0 / vehicle_mass;
            // cout << mass << "\t" << massFrac<<endl;
            grav = gravitational_acceleration;
            len = cable_length;
            za = offsetZ;
            J00 = Jxx;
            J11 = Jyy;
            J22 = Jzz;
            return;
        }

        inline void Forward(const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jer,
                            const Eigen::Vector3d &snp,
                            const Eigen::Vector3d &q,
                            const Eigen::Vector3d &dq,
                            const Eigen::Vector3d &ddq,
                            const Eigen::Vector3d &dFs,
                            const double &psi,
                            const double &dpsi,
                            const double &ddpsi,
                            Eigen::Vector4d &ftau,
                            double thrust,
                            Eigen::Vector4d &quat,
                            Eigen::Vector3d &omg)
        {
            F = dFs(0);
            dF = dFs(1);
            ddF = dFs(2);
            q0 = q(0);
            q1 = q(1);
            q2 = q(2);
            dq0 = dq(0);
            dq1 = dq(1);
            dq2 = dq(2);
            ddq0 = ddq(0);
            ddq1 = ddq(1);
            ddq2 = ddq(2);
            zu0 = acc(0) + massFrac * F * q0;
            zu1 = acc(1) + massFrac * F * q1;
            zu2 = acc(2) + massFrac * F * q2 + grav;

            zu_sqr_norm = zu0 * zu0 + zu1 * zu1 + zu2 * zu2;
            zu_norm = sqrt(zu_sqr_norm);
            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;
            zu_norm3 = zu_sqr_norm * zu_norm;
            zu_norm5 = zu_sqr_norm * zu_sqr_norm * zu_norm;

            dzu0 = jer(0) + massFrac * (F * dq0 + dF * q0);
            dzu1 = jer(1) + massFrac * (F * dq1 + dF * q1);
            dzu2 = jer(2) + massFrac * (F * dq2 + dF * q2);

            ddzu0 = snp(0) + massFrac * (F * ddq0 + 2 * dF * dq0 + ddF * q0);
            ddzu1 = snp(1) + massFrac * (F * ddq1 + 2 * dF * dq1 + ddF * q1);
            ddzu2 = snp(2) + massFrac * (F * ddq2 + 2 * dF * dq2 + ddF * q2);

            zudzu = zu0 * dzu0 + zu1 * dzu1 + zu2 * dzu2;
            dzudzu = dzu0 * dzu0 + dzu1 * dzu1 + dzu2 * dzu2;
            zuddzu = zu0 * ddzu0 + zu1 * ddzu1 + zu2 * ddzu2;

            dz0 = dzu0 / zu_norm - (zudzu * zu0) / zu_norm3;
            dz1 = dzu1 / zu_norm - (zudzu * zu1) / zu_norm3;
            dz2 = dzu2 / zu_norm - (zudzu * zu2) / zu_norm3;
            ddz0 = ddzu0 / zu_norm - ((dzudzu + zuddzu) * zu0 + 2 * zudzu * dzu0) / zu_norm3 + 3 * zudzu * zudzu * zu0 / zu_norm5;
            ddz1 = ddzu1 / zu_norm - ((dzudzu + zuddzu) * zu1 + 2 * zudzu * dzu1) / zu_norm3 + 3 * zudzu * zudzu * zu1 / zu_norm5;
            ddz2 = ddzu2 / zu_norm - ((dzudzu + zuddzu) * zu2 + 2 * zudzu * dzu2) / zu_norm3 + 3 * zudzu * zudzu * zu2 / zu_norm5;
            // cout << "forward\t"  <<  ddz1 << "\t" << ddz2  << endl; 

            ftau(0) = mass * zu_norm;

            tilt_den = sqrt(2.0 * (1.0 + z2));
            tilt0 = 0.5 * tilt_den;
            tilt1 = -z1 / tilt_den;
            tilt2 = z0 / tilt_den;
            c_half_psi = cos(0.5 * psi);
            s_half_psi = sin(0.5 * psi);
            quat0 = tilt0 * c_half_psi;
            quat1 = tilt1 * c_half_psi + tilt2 * s_half_psi;
            quat2 = tilt2 * c_half_psi - tilt1 * s_half_psi;
            quat3 = tilt0 * s_half_psi;

            R00 = 1 - 2 * quat2 * quat2 - 2 * quat3 * quat3;
            R10 = 2 * quat1 * quat2 + 2 * quat0 * quat3;
            R20 = 2 * quat1 * quat3 - 2 * quat0 * quat2;
            R01 = 2 * quat1 * quat2 - 2 * quat0 * quat3;
            R11 = 1 - 2 * quat1 * quat1 - 2 * quat3 * quat3;
            R21 = 2 * quat2 * quat3 + 2 * quat0 * quat1;

            c_psi = cos(psi);
            s_psi = sin(psi);
            omg_den = z2 + 1.0;

            omg0 = dz0 * s_psi - dz1 * c_psi - dz2 * (z0 * s_psi - z1 * c_psi) / omg_den;
            omg1 = dz0 * c_psi + dz1 * s_psi - dz2 * (z0 * c_psi + z1 * s_psi) / omg_den;
            omg2 = (z1 * dz0 - z0 * dz1) / omg_den;

            domg0 = ddz0 * s_psi - ddz1 * c_psi + dz0 * c_psi * dpsi + dz1 * s_psi * dpsi - s_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den + c_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * s_psi - z1 * c_psi) / (omg_den * omg_den);
            domg1 = ddz0 * c_psi + ddz1 * s_psi - dz0 * s_psi * dpsi + dz1 * c_psi * dpsi - c_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den - s_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * c_psi + z1 * s_psi) / (omg_den * omg_den);
            domg2 = (z1 * ddz0 - z0 * ddz1) / omg_den - dz2 * (z1 * dz0 - z0 * dz1) / (omg_den * omg_den);

            ftau(1) = J00 * domg0 + (J22 - J11) * omg1 * omg2 - za * F * (R01 * q0 + R11 * q1 + R21 * q2);
            // cout << "tau1\t" << fabs(J00 * domg0) / (fabs(J00 * domg0) + fabs((J22 - J11) * omg1 * omg2) + fabs(- za * F * (R01 * q0 + R11 * q1 + R21 * q2))) * 100
            //      << "\t" << fabs((J22 - J11) * omg1 * omg2) / (fabs(J00 * domg0) + fabs((J22 - J11) * omg1 * omg2) + fabs(- za * F * (R01 * q0 + R11 * q1 + R21 * q2))) * 100
            //      << "\t" << fabs(- za * F * (R01 * q0 + R11 * q1 + R21 * q2)) / (fabs(J00 * domg0) + fabs((J22 - J11) * omg1 * omg2) + fabs(- za * F * (R01 * q0 + R11 * q1 + R21 * q2))) * 100 << endl;

            ftau(2) = J11 * domg1 + (J00 - J22) * omg0 * omg2 + za * F * (R00 * q0 + R10 * q1 + R20 * q2);
            // cout << "tau2\t" << fabs(J11 * domg1) / (fabs(J11 * domg1) + fabs((J00 - J22) * omg0 * omg2) + fabs(za * F * (R00 * q0 + R10 * q1 + R20 * q2))) * 100
            //      << "\t" << fabs((J00 - J22) * omg0 * omg2) / (fabs(J11 * domg1) + fabs((J00 - J22) * omg0 * omg2) + fabs(za * F * (R00 * q0 + R10 * q1 + R20 * q2))) * 100
            //      << "\t" << fabs(za * F * (R00 * q0 + R10 * q1 + R20 * q2)) / (fabs(J11 * domg1) + fabs((J00 - J22) * omg0 * omg2) + fabs(za * F * (R00 * q0 + R10 * q1 + R20 * q2))) * 100<< endl;

            ftau(3) = J22 * domg2 + (J11 - J00) * omg1 * omg0;

            thr = ftau(0);
            tau0 = ftau(1);
            tau1 = ftau(2);
            tau2 = ftau(3);
            thrust = thr;
            omg(0) = omg0;
            omg(1) = omg1;
            omg(2) = omg2;
            quat(0) = quat0;
            quat(1) = quat1;
            quat(2) = quat2;
            quat(3) = quat3;
            return;
        }

        inline void Backward(const bool isCalFlatness,
                             const Eigen::Vector3d &q_grad,
                             const double &F_grad,
                             const Eigen::Vector4d &ftau_grad,
                             Eigen::Vector3d &q_total_grad,
                             Eigen::Vector3d &dq_total_grad,
                             Eigen::Vector3d &d2q_total_grad,
                             Eigen::Vector3d &d3q_total_grad,
                             Eigen::Vector3d &d4q_total_grad,
                             Eigen::Vector3d &F_total_grads,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad,
                             Eigen::Vector3d &snp_total_grad)
        {
            if (isCalFlatness)
            {
                // cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;
                // domg2b = J22 * ftau_grad(3);
                // omg1b = omg0 * (J11 - J00) * ftau_grad(3) + omg2 * (J22 - J11) * ftau_grad(1);
                // omg0b = omg1 * (J11 - J00) * ftau_grad(3) + omg2 * (J00 - J22) * ftau_grad(2);
                // domg1b = J11 * ftau_grad(2);
                // omg2b = omg0 * (J00 - J22) * ftau_grad(2) + omg1 * (J22 - J11) * ftau_grad(1);
                // tempb1 = F * za * ftau_grad(2);
                // R00b = q0 * tempb1;
                // q0b = R00 * tempb1;
                // R10b = q1 * tempb1;
                // q1b = R10 * tempb1;
                // R20b = q2 * tempb1;
                // q2b = R20 * tempb1;
                // domg0b = J00 * ftau_grad(1);
                // tempb1 = -(F * za * ftau_grad(1));
                // R01b = q0 * tempb1;
                // q0b = q0b + R01 * tempb1;
                // R11b = q1 * tempb1;
                // q1b = q1b + R11 * tempb1;
                // R21b = q2 * tempb1;
                // q2b = q2b + R21 * tempb1;
                // temp = dz2 / (omg_den * omg_den);
                // tempb1 = domg2b / omg_den;
                // tempb = -(temp * domg2b);
                // tempb0 = -((z1 * dz0 - z0 * dz1) * domg2b / (omg_den * omg_den));
                // dz2b = tempb0;
                // omg_denb = -(2 * omg_den * temp * tempb0) - (z1 * ddz0 - z0 * ddz1) * tempb1 / omg_den;
                // z1b = dz0 * tempb;
                // dz0b = z1 * tempb;
                // z0b = -(dz1 * tempb) - ddz1 * tempb1;
                // dz1b = -(z0 * tempb);
                // ddz0b = z1 * tempb1 + c_psi * domg1b + s_psi * domg0b;
                // ddz1b = s_psi * domg1b - z0 * tempb1 - c_psi * domg0b;
                // temp = dz2 * dz2 / (omg_den * omg_den);
                // tempb0 = (c_psi * z0 + s_psi * z1) * domg1b / (omg_den * omg_den);
                // tempb = -(s_psi * domg1b / omg_den);
                // z1b = z1b + ddz0 * tempb1 + s_psi * temp * domg1b + ddz2 * tempb;
                // tempb1 = -(c_psi * domg1b / omg_den);
                // ddz2b = z1 * tempb;
                // dz2b = dz2b + dz1 * tempb + 2 * dz2 * tempb0 + dz0 * tempb1;
                // dz1b = dz1b + dz2 * tempb;
                // omg_denb = omg_denb - (ddz2 * z1 + dz2 * dz1) * tempb / omg_den - 2 * omg_den * temp * tempb0 - (ddz2 * z0 + dz2 * dz0) * tempb1 / omg_den;
                // temp0 = dz2 * dz2 / (omg_den * omg_den);
                // tempb0 = -(s_psi * domg0b / omg_den);
                // z0b = z0b + c_psi * temp * domg1b + ddz2 * tempb1 + s_psi * temp0 * domg0b + ddz2 * tempb0;
                // dz0b = dz0b + dz2 * tempb1 + dz2 * tempb0;
                // tempb = c_psi * domg0b / omg_den;
                // ddz2b = ddz2b + z0 * tempb1 + z1 * tempb + z0 * tempb0;
                // z1b = z1b + ddz2 * tempb - c_psi * temp0 * domg0b;
                // tempb1 = (s_psi * z0 - c_psi * z1) * domg0b / (omg_den * omg_den);
                // dz2b = dz2b + 2 * dz2 * tempb1 + dz1 * tempb;
                // omg_denb = omg_denb - 2 * omg_den * temp0 * tempb1 - (ddz2 * z1 + dz2 * dz1) * tempb / omg_den - (ddz2 * z0 + dz2 * dz0) * tempb0 / omg_den;
                // dz1b = dz1b + dz2 * tempb;
                // tempb = -((c_psi * z0 + s_psi * z1) * omg1b);
                // dz2b = dz2b + dz0 * tempb0 + omg_den * tempb;
                // tempb0 = omg2b / omg_den;
                // z1b = z1b + dz0 * tempb0;
                // dz0b = dz0b + z1 * tempb0 + c_psi * omg1b + s_psi * omg0b;
                // z0b = z0b - dz1 * tempb0;
                // dz1b = dz1b + s_psi * omg1b - z0 * tempb0 - c_psi * omg0b;
                // omg_denb = omg_denb + dz2 * tempb - (z1 * dz0 - z0 * dz1) * tempb0 / omg_den;
                // tempb0 = -(dz2 * omg_den * omg1b);
                // z0b = z0b + c_psi * tempb0;
                // z1b = z1b + s_psi * tempb0;
                // tempb0 = -(dz2 * omg_den * omg0b);
                // tempb = -((s_psi * z0 - c_psi * z1) * omg0b);
                // dz2b = dz2b + omg_den * tempb;
                // omg_denb = omg_denb + dz2 * tempb;
                // quat2b = 2 * quat3 * R21b + 2 * quat1 * R01b + 2 * quat1 * R10b - 2 * quat0 * R20b - 2 * 2 * quat2 * R00b;
                // quat3b = 2 * quat2 * R21b + 2 * quat1 * R20b - 2 * 2 * quat3 * R11b - 2 * quat0 * R01b + 2 * quat0 * R10b - 2 * 2 * quat3 * R00b;
                // quat0b = 2 * quat1 * R21b + 2 * quat3 * R10b - 2 * quat3 * R01b - 2 * quat2 * R20b;
                // quat1b = 2 * quat0 * R21b + 2 * quat2 * R01b - 2 * 2 * quat1 * R11b + 2 * quat3 * R20b + 2 * quat2 * R10b;
                // tilt0b = s_half_psi * quat3b + c_half_psi * quat0b;
                // tilt2b = c_half_psi * quat2b + s_half_psi * quat1b;
                // z0b = z0b + s_psi * tempb0 + tilt2b / tilt_den;
                // tilt1b = c_half_psi * quat1b - s_half_psi * quat2b;
                // z1b = z1b - c_psi * tempb0 - tilt1b / tilt_den;
                // tilt_denb = z1 * tilt1b / (tilt_den * tilt_den) - z0 * tilt2b / (tilt_den * tilt_den) + 0.5 * tilt0b;
                // z2b = omg_denb + tilt_denb / sqrt(2.0 * (z2 + 1.0));
                // tempb0 = -(ddz2b / zu_norm3);
                // zudzub = 2 * zudzu * 3 * zu2 * ddz2b / zu_norm5 + 2 * dzu2 * tempb0;
                // tempb = 3 * (zudzu * zudzu) * ddz2b / zu_norm5;
                // zu2b = tempb + (dzudzu + zuddzu) * tempb0;
                // zu_norm5b = -(zu2 * tempb / zu_norm5);
                // dzudzub = zu2 * tempb0;
                // zuddzub = zu2 * tempb0;
                // dzu2b = 2 * zudzu * tempb0;
                // zu_norm3b = -(((dzudzu + zuddzu) * zu2 + 2 * zudzu * dzu2) * tempb0 / zu_norm3);
                // tempb0 = -(ddz1b / zu_norm3);
                // tempb = 3 * (zudzu * zudzu) * ddz1b / zu_norm5;
                // zu1b = tempb + (dzudzu + zuddzu) * tempb0;
                // zu_norm5b = zu_norm5b - zu1 * tempb / zu_norm5;
                // dzu1b = 2 * zudzu * tempb0;
                // tempb = -(ddz0b / zu_norm3);
                // zudzub = zudzub + 2 * zudzu * 3 * zu1 * ddz1b / zu_norm5 + 2 * dzu1 * tempb0 + 2 * zudzu * 3 * zu0 * ddz0b / zu_norm5 + 2 * dzu0 * tempb;
                // dzudzub = dzudzub + zu1 * tempb0 + zu0 * tempb;
                // zuddzub = zuddzub + zu1 * tempb0 + zu0 * tempb;
                // ddzu2b = ddz2b / zu_norm + zu2 * zuddzub;
                // ddzu1b = ddz1b / zu_norm + zu1 * zuddzub;
                // zu_norm3b = zu_norm3b - ((dzudzu + zuddzu) * zu1 + 2 * zudzu * dzu1) * tempb0 / zu_norm3 - ((dzudzu + zuddzu) * zu0 + 2 * zudzu * dzu0) * tempb / zu_norm3;
                // ddzu0b = ddz0b / zu_norm + zu0 * zuddzub;
                // tempb0 = 3 * (zudzu * zudzu) * ddz0b / zu_norm5;
                // zu0b = tempb0 + (dzudzu + zuddzu) * tempb;
                // zu_norm5b = zu_norm5b - zu0 * tempb0 / zu_norm5;
                // dzu0b = 2 * zudzu * tempb;
                // tempb = -(dz2b / zu_norm3);
                // zudzub = zudzub + zu2 * tempb;
                // zu2b = zu2b + zudzu * tempb;
                // zu_norm3b = zu_norm3b - zudzu * zu2 * tempb / zu_norm3;
                // tempb = -(dz1b / zu_norm3);
                // zudzub = zudzub + zu1 * tempb;
                // zu1b = zu1b + zudzu * tempb;
                // zu_norm3b = zu_norm3b - zudzu * zu1 * tempb / zu_norm3;
                // tempb = -(dz0b / zu_norm3);
                // zudzub = zudzub + zu0 * tempb;
                // dzu2b = dzu2b + dz2b / zu_norm + 2 * dzu2 * dzudzub + zu2 * zudzub;
                // dzu1b = dzu1b + dz1b / zu_norm + 2 * dzu1 * dzudzub + zu1 * zudzub;
                // dzu0b = dzu0b + dz0b / zu_norm + 2 * dzu0 * dzudzub + zu0 * zudzub;
                // zu_norm3b = zu_norm3b - zudzu * zu0 * tempb / zu_norm3;
                // zu_normb = mass * ftau_grad(0) + zu_sqr_norm * zu_sqr_norm * zu_norm5b - ddzu2 * ddz2b / (zu_norm * zu_norm) - ddzu1 * ddz1b / (zu_norm * zu_norm) - ddzu0 * ddz0b / (zu_norm * zu_norm) - dzu2 * dz2b / (zu_norm * zu_norm) - dzu1 * dz1b / (zu_norm * zu_norm) - dzu0 * dz0b / (zu_norm * zu_norm) + zu_sqr_norm * zu_norm3b - zu2 * z2b / (zu_norm * zu_norm) - zu1 * z1b / (zu_norm * zu_norm) - zu0 * z0b / (zu_norm * zu_norm);
                // s2b = ddzu2b;
                // s1b = ddzu1b;
                // s0b = ddzu0b;
                // j2b = dzu2b;
                // j1b = dzu1b;
                // j0b = dzu0b;
                // zu_sqr_normb = 2 * zu_sqr_norm * zu_norm * zu_norm5b + zu_norm * zu_norm3b + zu_normb / (2.0 * sqrt(zu_sqr_norm));
                // zu0b = zu0b + zudzu * tempb + ddzu0 * zuddzub + dzu0 * zudzub + z0b / zu_norm + 2 * zu0 * zu_sqr_normb;
                // zu1b = zu1b + ddzu1 * zuddzub + dzu1 * zudzub + z1b / zu_norm + 2 * zu1 * zu_sqr_normb;
                // zu2b = zu2b + ddzu2 * zuddzub + dzu2 * zudzub + z2b / zu_norm + 2 * zu2 * zu_sqr_normb;
                // tempb = massFrac * ddzu2b;
                // Fb = (R00 * q0 + R10 * q1 + R20 * q2) * za * ftau_grad(2) + ddq2 * tempb - (R01 * q0 + R11 * q1 + R21 * q2) * za * ftau_grad(1);
                // ddq2b = F * tempb;
                // dFb = 2 * dq2 * tempb;
                // dq2b = 2 * dF * tempb;
                // ddFb = q2 * tempb;
                // q2b = q2b + ddF * tempb;
                // tempb = massFrac * ddzu1b;
                // Fb = Fb + ddq1 * tempb;
                // ddq1b = F * tempb;
                // dFb = dFb + 2 * dq1 * tempb;
                // dq1b = 2 * dF * tempb;
                // ddFb = ddFb + q1 * tempb;
                // q1b = q1b + ddF * tempb;
                // tempb = massFrac * ddzu0b;
                // Fb = Fb + ddq0 * tempb;
                // ddq0b = F * tempb;
                // dFb = dFb + 2 * dq0 * tempb;
                // dq0b = 2 * dF * tempb;
                // ddFb = ddFb + q0 * tempb;
                // q0b = q0b + ddF * tempb;
                // tempb = massFrac * dzu2b;
                // Fb = Fb + dq2 * tempb;
                // dq2b = dq2b + F * tempb;
                // dFb = dFb + q2 * tempb;
                // q2b = q2b + dF * tempb + F * massFrac * zu2b;
                // tempb = massFrac * dzu1b;
                // Fb = Fb + dq1 * tempb;
                // dq1b = dq1b + F * tempb;
                // dFb = dFb + q1 * tempb;
                // q1b = q1b + dF * tempb + F * massFrac * zu1b;
                // tempb = massFrac * dzu0b;
                // Fb = Fb + dq0 * tempb + q2 * massFrac * zu2b + q1 * massFrac * zu1b + q0 * massFrac * zu0b;
                // dq0b = dq0b + F * tempb;
                // dFb = dFb + q0 * tempb;
                // q0b = q0b + dF * tempb + F * massFrac * zu0b;
                // a2b = zu2b;
                // a1b = zu1b;
                // a0b = zu0b;

                // snp_total_grad(2) += s2b;
                // snp_total_grad(1) += s1b;
                // snp_total_grad(0) += s0b;
                // jer_total_grad(2) += j2b;
                // jer_total_grad(1) += j1b;
                // jer_total_grad(0) += j0b;
                // acc_total_grad(2) += a2b;
                // acc_total_grad(1) += a1b;
                // acc_total_grad(0) += a0b;

                // d4q_total_grad(0) = len * s0b;
                // d4q_total_grad(1) = len * s1b;
                // d4q_total_grad(2) = len * s2b;
                // d3q_total_grad(0) = len * j0b;
                // d3q_total_grad(1) = len * j1b;
                // d3q_total_grad(2) = len * j2b;
                // d2q_total_grad(0) = len * a0b + ddq0b;
                // d2q_total_grad(1) = len * a1b + ddq1b;
                // d2q_total_grad(2) = len * a2b + ddq2b;
                // dq_total_grad(0) = dq0b;
                // dq_total_grad(1) = dq1b;
                // dq_total_grad(2) = dq2b;
                // q_total_grad(0) = q_grad(0) + q0b;
                // q_total_grad(1) = q_grad(1) + q1b;
                // q_total_grad(2) = q_grad(2) + q2b;
                // F_total_grads(2) = ddFb;
                // F_total_grads(1) = dFb;
                // F_total_grads(0) = Fb + F_grad;

                domg2b = J22 * ftau_grad(3);
                omg1b = omg0 * (J11 - J00) * ftau_grad(3) + omg2 * (J22 - J11) * ftau_grad(1);
                omg0b = omg1 * (J11 - J00) * ftau_grad(3) + omg2 * (J00 - J22) * ftau_grad(2);
                domg1b = J11 * ftau_grad(2);
                omg2b = omg0 * (J00 - J22) * ftau_grad(2) + omg1 * (J22 - J11) * ftau_grad(1);
                tempb1 = F * za * ftau_grad(2);
                R00b = q0 * tempb1;
                q0b = R00 * tempb1;
                R10b = q1 * tempb1;
                q1b = R10 * tempb1;
                R20b = q2 * tempb1;
                q2b = R20 * tempb1;
                domg0b = J00 * ftau_grad(1);
                tempb1 = -(F * za * ftau_grad(1));
                R01b = q0 * tempb1;
                q0b = q0b + R01 * tempb1;
                R11b = q1 * tempb1;
                q1b = q1b + R11 * tempb1;
                R21b = q2 * tempb1;
                q2b = q2b + R21 * tempb1;
                temp = dz2 / (omg_den * omg_den);
                tempb1 = domg2b / omg_den;
                tempb = -(temp * domg2b);
                tempb0 = -((z1 * dz0 - z0 * dz1) * domg2b / (omg_den * omg_den));
                dz2b = tempb0;
                omg_denb = -(2 * omg_den * temp * tempb0) - (z1 * ddz0 - z0 * ddz1) * tempb1 / omg_den;
                z1b = dz0 * tempb;
                dz0b = z1 * tempb;
                z0b = -(dz1 * tempb) - ddz1 * tempb1;
                dz1b = -(z0 * tempb);
                ddz0b = z1 * tempb1 + c_psi * domg1b + s_psi * domg0b;
                ddz1b = s_psi * domg1b - z0 * tempb1 - c_psi * domg0b;
                temp = dz2 * dz2 / (omg_den * omg_den);
                tempb0 = (c_psi * z0 + s_psi * z1) * domg1b / (omg_den * omg_den);
                tempb = -(s_psi * domg1b / omg_den);
                z1b = z1b + ddz0 * tempb1 + s_psi * temp * domg1b + ddz2 * tempb;
                tempb1 = -(c_psi * domg1b / omg_den);
                ddz2b = z1 * tempb;
                dz2b = dz2b + dz1 * tempb + 2 * dz2 * tempb0 + dz0 * tempb1;
                dz1b = dz1b + dz2 * tempb;
                omg_denb = omg_denb - (ddz2 * z1 + dz2 * dz1) * tempb / omg_den - 2 * omg_den * temp * tempb0 - (ddz2 * z0 + dz2 * dz0) * tempb1 / omg_den;
                temp0 = dz2 * dz2 / (omg_den * omg_den);
                tempb0 = -(s_psi * domg0b / omg_den);
                z0b = z0b + c_psi * temp * domg1b + ddz2 * tempb1 + s_psi * temp0 * domg0b + ddz2 * tempb0;
                dz0b = dz0b + dz2 * tempb1 + dz2 * tempb0;
                tempb = c_psi * domg0b / omg_den;
                ddz2b = ddz2b + z0 * tempb1 + z1 * tempb + z0 * tempb0;
                tempb1 = (s_psi * z0 - c_psi * z1) * domg0b / (omg_den * omg_den);
                dz2b = dz2b + 2 * dz2 * tempb1 + dz1 * tempb + dz0 * tempb0;
                omg_denb = omg_denb - 2 * omg_den * temp0 * tempb1 - (ddz2 * z1 + dz2 * dz1) * tempb / omg_den - (ddz2 * z0 + dz2 * dz0) * tempb0 / omg_den;
                tempb0 = omg2b / omg_den;
                dz1b = dz1b + dz2 * tempb + s_psi * omg1b - z0 * tempb0 - c_psi * omg0b;
                dz0b = dz0b + z1 * tempb0 + c_psi * omg1b + s_psi * omg0b;
                omg_denb = omg_denb - (z1 * dz0 - z0 * dz1) * tempb0 / omg_den;
                temp = dz2 / omg_den;
                z1b = z1b + ddz2 * tempb - c_psi * temp0 * domg0b + dz0 * tempb0 - s_psi * temp * omg1b;
                z0b = z0b - dz1 * tempb0 - c_psi * temp * omg1b;
                tempb0 = -((c_psi * z0 + s_psi * z1) * omg1b / omg_den);
                dz2b = dz2b + tempb0;
                omg_denb = omg_denb - temp * tempb0;
                temp = dz2 / omg_den;
                tempb0 = -((s_psi * z0 - c_psi * z1) * omg0b / omg_den);
                dz2b = dz2b + tempb0;
                omg_denb = omg_denb - temp * tempb0;
                quat2b = 2 * quat3 * R21b + 2 * quat1 * R01b + 2 * quat1 * R10b - 2 * quat0 * R20b - 2 * 2 * quat2 * R00b;
                quat3b = 2 * quat2 * R21b + 2 * quat1 * R20b - 2 * 2 * quat3 * R11b - 2 * quat0 * R01b + 2 * quat0 * R10b - 2 * 2 * quat3 * R00b;
                quat0b = 2 * quat1 * R21b + 2 * quat3 * R10b - 2 * quat3 * R01b - 2 * quat2 * R20b;
                quat1b = 2 * quat0 * R21b + 2 * quat2 * R01b - 2 * 2 * quat1 * R11b + 2 * quat3 * R20b + 2 * quat2 * R10b;
                tilt0b = s_half_psi * quat3b + c_half_psi * quat0b;
                tilt2b = c_half_psi * quat2b + s_half_psi * quat1b;
                z0b = z0b + tilt2b / tilt_den - s_psi * temp * omg0b;
                tilt1b = c_half_psi * quat1b - s_half_psi * quat2b;
                z1b = z1b + c_psi * temp * omg0b - tilt1b / tilt_den;
                tilt_denb = z1 * tilt1b / (tilt_den * tilt_den) - z0 * tilt2b / (tilt_den * tilt_den) + 0.5 * tilt0b;
                z2b = omg_denb + tilt_denb / sqrt(2.0 * (z2 + 1.0));
                tempb0 = -(ddz2b / zu_norm3);
                zudzub = 2 * zudzu * 3 * zu2 * ddz2b / zu_norm5 + 2 * dzu2 * tempb0;
                tempb = 3 * (zudzu * zudzu) * ddz2b / zu_norm5;
                zu2b = tempb + (dzudzu + zuddzu) * tempb0;
                zu_norm5b = -(zu2 * tempb / zu_norm5);
                dzudzub = zu2 * tempb0;
                zuddzub = zu2 * tempb0;
                dzu2b = 2 * zudzu * tempb0;
                zu_norm3b = -(((dzudzu + zuddzu) * zu2 + 2 * zudzu * dzu2) * tempb0 / zu_norm3);
                tempb0 = -(ddz1b / zu_norm3);
                tempb = 3 * (zudzu * zudzu) * ddz1b / zu_norm5;
                zu1b = tempb + (dzudzu + zuddzu) * tempb0;
                zu_norm5b = zu_norm5b - zu1 * tempb / zu_norm5;
                dzu1b = 2 * zudzu * tempb0;
                tempb = -(ddz0b / zu_norm3);
                zudzub = zudzub + 2 * zudzu * 3 * zu1 * ddz1b / zu_norm5 + 2 * dzu1 * tempb0 + 2 * zudzu * 3 * zu0 * ddz0b / zu_norm5 + 2 * dzu0 * tempb;
                dzudzub = dzudzub + zu1 * tempb0 + zu0 * tempb;
                zuddzub = zuddzub + zu1 * tempb0 + zu0 * tempb;
                ddzu2b = ddz2b / zu_norm + zu2 * zuddzub;
                ddzu1b = ddz1b / zu_norm + zu1 * zuddzub;
                zu_norm3b = zu_norm3b - ((dzudzu + zuddzu) * zu1 + 2 * zudzu * dzu1) * tempb0 / zu_norm3 - ((dzudzu + zuddzu) * zu0 + 2 * zudzu * dzu0) * tempb / zu_norm3;
                ddzu0b = ddz0b / zu_norm + zu0 * zuddzub;
                tempb0 = 3 * (zudzu * zudzu) * ddz0b / zu_norm5;
                zu0b = tempb0 + (dzudzu + zuddzu) * tempb;
                zu_norm5b = zu_norm5b - zu0 * tempb0 / zu_norm5;
                dzu0b = 2 * zudzu * tempb;
                tempb = -(dz2b / zu_norm3);
                zudzub = zudzub + zu2 * tempb;
                zu2b = zu2b + zudzu * tempb;
                zu_norm3b = zu_norm3b - zudzu * zu2 * tempb / zu_norm3;
                tempb = -(dz1b / zu_norm3);
                zudzub = zudzub + zu1 * tempb;
                zu1b = zu1b + zudzu * tempb;
                zu_norm3b = zu_norm3b - zudzu * zu1 * tempb / zu_norm3;
                tempb = -(dz0b / zu_norm3);
                zudzub = zudzub + zu0 * tempb;
                dzu2b = dzu2b + dz2b / zu_norm + 2 * dzu2 * dzudzub + zu2 * zudzub;
                dzu1b = dzu1b + dz1b / zu_norm + 2 * dzu1 * dzudzub + zu1 * zudzub;
                dzu0b = dzu0b + dz0b / zu_norm + 2 * dzu0 * dzudzub + zu0 * zudzub;
                zu_norm3b = zu_norm3b - zudzu * zu0 * tempb / zu_norm3;
                zu_normb = mass * ftau_grad(0) + zu_sqr_norm * zu_sqr_norm * zu_norm5b - ddzu2 * ddz2b / (zu_norm * zu_norm) - ddzu1 * ddz1b / (zu_norm * zu_norm) - ddzu0 * ddz0b / (zu_norm * zu_norm) - dzu2 * dz2b / (zu_norm * zu_norm) - dzu1 * dz1b / (zu_norm * zu_norm) - dzu0 * dz0b / (zu_norm * zu_norm) + zu_sqr_norm * zu_norm3b - zu2 * z2b / (zu_norm * zu_norm) - zu1 * z1b / (zu_norm * zu_norm) - zu0 * z0b / (zu_norm * zu_norm);
                s2b = ddzu2b;
                s1b = ddzu1b;
                s0b = ddzu0b;
                j2b = dzu2b;
                j1b = dzu1b;
                j0b = dzu0b;
                zu_sqr_normb = 2 * zu_sqr_norm * zu_norm * zu_norm5b + zu_norm * zu_norm3b + zu_normb / (2.0 * sqrt(zu_sqr_norm));
                zu0b = zu0b + zudzu * tempb + ddzu0 * zuddzub + dzu0 * zudzub + z0b / zu_norm + 2 * zu0 * zu_sqr_normb;
                zu1b = zu1b + ddzu1 * zuddzub + dzu1 * zudzub + z1b / zu_norm + 2 * zu1 * zu_sqr_normb;
                zu2b = zu2b + ddzu2 * zuddzub + dzu2 * zudzub + z2b / zu_norm + 2 * zu2 * zu_sqr_normb;
                tempb = massFrac * ddzu2b;
                Fb = (R00 * q0 + R10 * q1 + R20 * q2) * za * ftau_grad(2) + ddq2 * tempb - (R01 * q0 + R11 * q1 + R21 * q2) * za * ftau_grad(1);
                ddq2b = F * tempb;
                dFb = 2 * dq2 * tempb;
                dq2b = 2 * dF * tempb;
                ddFb = q2 * tempb;
                q2b = q2b + ddF * tempb;
                tempb = massFrac * ddzu1b;
                Fb = Fb + ddq1 * tempb;
                ddq1b = F * tempb;
                dFb = dFb + 2 * dq1 * tempb;
                dq1b = 2 * dF * tempb;
                ddFb = ddFb + q1 * tempb;
                q1b = q1b + ddF * tempb;
                tempb = massFrac * ddzu0b;
                Fb = Fb + ddq0 * tempb;
                ddq0b = F * tempb;
                dFb = dFb + 2 * dq0 * tempb;
                dq0b = 2 * dF * tempb;
                ddFb = ddFb + q0 * tempb;
                q0b = q0b + ddF * tempb;
                tempb = massFrac * dzu2b;
                Fb = Fb + dq2 * tempb;
                dq2b = dq2b + F * tempb;
                dFb = dFb + q2 * tempb;
                q2b = q2b + dF * tempb + F * massFrac * zu2b;
                tempb = massFrac * dzu1b;
                Fb = Fb + dq1 * tempb;
                dq1b = dq1b + F * tempb;
                dFb = dFb + q1 * tempb;
                q1b = q1b + dF * tempb + F * massFrac * zu1b;
                tempb = massFrac * dzu0b;
                Fb = Fb + dq0 * tempb + q2 * massFrac * zu2b + q1 * massFrac * zu1b + q0 * massFrac * zu0b;
                dq0b = dq0b + F * tempb;
                dFb = dFb + q0 * tempb;
                q0b = q0b + dF * tempb + F * massFrac * zu0b;
                a2b = zu2b;
                a1b = zu1b;
                a0b = zu0b; 
                // cout << "backward\t" << ddz1 << "\t" << ddz2  << endl; 

                snp_total_grad(2) += s2b;
                snp_total_grad(1) += s1b;
                snp_total_grad(0) += s0b;
                jer_total_grad(2) += j2b;
                jer_total_grad(1) += j1b;
                jer_total_grad(0) += j0b;
                acc_total_grad(2) += a2b;
                acc_total_grad(1) += a1b;
                acc_total_grad(0) += a0b;

                d4q_total_grad(0) = len * s0b;
                d4q_total_grad(1) = len * s1b;
                d4q_total_grad(2) = len * s2b;
                d3q_total_grad(0) = len * j0b;
                d3q_total_grad(1) = len * j1b;
                d3q_total_grad(2) = len * j2b;
                d2q_total_grad(0) = len * a0b + ddq0b;
                d2q_total_grad(1) = len * a1b + ddq1b;
                d2q_total_grad(2) = len * a2b + ddq2b;
                dq_total_grad(0) = dq0b;
                dq_total_grad(1) = dq1b;
                dq_total_grad(2) = dq2b;
                q_total_grad(0) = q_grad(0) + q0b;
                q_total_grad(1) = q_grad(1) + q1b;
                q_total_grad(2) = q_grad(2) + q2b;
                F_total_grads(2) = ddFb;
                F_total_grads(1) = dFb;
                F_total_grads(0) = Fb + F_grad;
            }
            else
            {
                // snp_total_grad.setZero();
                // jer_total_grad.setZero();
                // acc_total_grad.setZero();
                d4q_total_grad.setZero();
                d3q_total_grad.setZero();
                d2q_total_grad.setZero();
                dq_total_grad.setZero();
                q_total_grad = q_grad;
                F_total_grads(2) = 0.0;
                F_total_grads(1) = 0.0;
                F_total_grads(0) = F_grad;
            }
        }

    private:
        double mass, massFrac, grav, len, J11, J22, J00, za;

        double a0b, a1b, a2b, s0b, s1b, s2b, j0b, j1b, j2b;
        double q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2, q0b, q1b, q2b, dq0b, dq1b, dq2b, ddq0b, ddq1b, ddq2b;
        double z0, z1, z2, dz0, dz1, dz2, ddz0, ddz1, ddz2, z0b, z1b, z2b, dz0b, dz1b, dz2b, ddz0b, ddz1b, ddz2b;
        double F, dF, ddF, Fb, dFb, ddFb;
        double zu_sqr_norm, zu_sqr_normb, zu_norm, zu_normb, zu0, zu1, zu2, zu0b, zu1b, zu2b;
        double zu_norm3, zu_norm5, zu_norm3b, zu_norm5b;
        double dzu0, dzu1, dzu2, dzu0b, dzu1b, dzu2b;
        double ddzu0, ddzu1, ddzu2, ddzu0b, ddzu1b, ddzu2b;
        double zudzu, dzudzu, zuddzu, zudzub, dzudzub, zuddzub;
        double thr, tau0, tau1, tau2;
        double R00, R01, R10, R11, R20, R21, R00b, R01b, R10b, R11b, R20b, R21b;
        double omg0, omg1, omg2, domg0, domg1, domg2, omg0b, omg1b, omg2b, domg0b, domg1b, domg2b;
        double quat0, quat1, quat2, quat3, quat0b, quat1b, quat2b, quat3b;
        double temp, tempb, temp0, tempb0, tempb1;
        double tilt_den, tilt_denb, tilt0, tilt1, tilt2, tilt0b, tilt1b, tilt2b, c_half_psi, s_half_psi;
        double c_psi, s_psi, omg_den, omg_denb;
    };
}

#endif
