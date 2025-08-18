#include <math.h>

double mass, massFrac, J11, J22, J00, grav, za;
double thr;
double a0, a1, a2, j0, j1, j2, s0, s1, s2, q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2, F, dF, ddF;
double psi, dpsi, ddpsi;
double quat0, quat1, quat2, quat3;
double omg0, omg1, omg2, domg0, domg1, domg2;
double z0, z1, z2, dz0, dz1, dz2, ddz0, ddz1, ddz2;
double zu_sqr_norm, zu_norm, zu_norm3, zu_norm5, zu0, zu1, zu2, dzu0, dzu1, dzu2, ddzu0, ddzu1, ddzu2;
double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
double c_psi, s_psi, omg_den;
double zudzu, zuddzu, dzudzu;
double tau0, tau1, tau2;
double R00, R01, R10, R11, R20, R21;

void forward( )
{
    zu0 = a0 + massFrac * F * q0;
    zu1 = a1 + massFrac * F * q1;
    zu2 = a2 + massFrac * F * q2 + grav;

    zu_sqr_norm = zu0 * zu0 + zu1 * zu1 + zu2 * zu2;
    zu_norm = sqrt(zu_sqr_norm);
    z0 = zu0 / zu_norm;
    z1 = zu1 / zu_norm;
    z2 = zu2 / zu_norm;
    zu_norm3 = zu_sqr_norm * zu_norm;
    zu_norm5 = zu_sqr_norm * zu_sqr_norm * zu_norm;

    dzu0 = j0 + massFrac * (F * dq0 + dF * q0);
    dzu1 = j1 + massFrac * (F * dq1 + dF * q1);
    dzu2 = j2 + massFrac * (F * dq2 + dF * q2);

    ddzu0 = s0 + massFrac * (F * ddq0 + 2 * dF * dq0 +  ddF * q0);
    ddzu1 = s1 + massFrac * (F * ddq1 + 2 * dF * dq1 +  ddF * q1);
    ddzu2 = s2 + massFrac * (F * ddq2 + 2 * dF * dq2 +  ddF * q2);

    zudzu = zu0 * dzu0 + zu1 * dzu1 + zu2 * dzu2;
    dzudzu = dzu0 * dzu0 + dzu1 * dzu1 + dzu2 * dzu2;
    zuddzu = zu0 * ddzu0 + zu1 * ddzu1 + zu2 * ddzu2;

    dz0 = dzu0 / zu_norm - (zudzu * zu0) / zu_norm3;
    dz1 = dzu1 / zu_norm - (zudzu * zu1) / zu_norm3;
    dz2 = dzu2 / zu_norm - (zudzu * zu2) / zu_norm3;
    ddz0 = ddzu0 / zu_norm - ((dzudzu + zuddzu) * zu0 + 2 * zudzu * dzu0) / zu_norm3 + 3 * zudzu * zudzu * zu0 / zu_norm5;
    ddz1 = ddzu1 / zu_norm - ((dzudzu + zuddzu) * zu1 + 2 * zudzu * dzu1) / zu_norm3 + 3 * zudzu * zudzu * zu1 / zu_norm5;
    ddz2 = ddzu2 / zu_norm - ((dzudzu + zuddzu) * zu2 + 2 * zudzu * dzu2) / zu_norm3 + 3 * zudzu * zudzu * zu2 / zu_norm5;

    thr = mass * zu_norm;

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

    omg0 = dz0 * s_psi - dz1 * c_psi - dz2 * (z0 * s_psi - z1 * c_psi) * omg_den;
    omg1 = dz0 * c_psi + dz1 * s_psi - dz2 * (z0 * c_psi + z1 * s_psi) * omg_den;
    omg2 = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;
    
    domg0 = ddz0 * s_psi - ddz1 * c_psi + dz0 * c_psi * dpsi + dz1 * s_psi * dpsi
            - s_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den + c_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * s_psi - z1 * c_psi) / (omg_den * omg_den);
    domg1 = ddz0 * c_psi + ddz1 * s_psi - dz0 * s_psi * dpsi + dz1 * c_psi * dpsi 
            - c_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den - s_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * c_psi + z1 * s_psi) / (omg_den * omg_den);
    domg2 = (z1 * ddz0 - z0 * ddz1) / omg_den - dz2 * (z1 * dz0 - z0 * dz1) / (omg_den * omg_den) + ddpsi;
    
    tau0 = J00 * domg0 + (J22 - J11) * omg1 * omg2 - za * F * (R01 * q0 + R11 * q1 + R21 * q2); 

    tau1 = J11 * domg1 + (J00 - J22) * omg0 * omg2 + za * F * (R00 * q0 + R10 * q1 + R20 * q2);
    
    tau2 = J22 * domg2 + (J11 - J00) * omg1 * omg0;

    return;
}