#include "core/ForceController.h"
#include "core/PoseOps.h"
#include <algorithm>
#include <cmath>

// ── Coordinate transform helpers ────────────────────────────────────

static void eulerToRotation(double aDeg, double bDeg, double cDeg, double R[3][3])
{
    using namespace poseops;
    const Quat q = quatFromABC(aDeg, bDeg, cDeg);
    // Convert unit quaternion to 3×3 rotation matrix
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    const double xx = x*x, yy = y*y, zz = z*z;
    const double wx = w*x, wy = w*y, wz = w*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    R[0][0] = 1.0 - 2.0*(yy + zz);  R[0][1] = 2.0*(xy - wz);       R[0][2] = 2.0*(xz + wy);
    R[1][0] = 2.0*(xy + wz);         R[1][1] = 1.0 - 2.0*(xx + zz); R[1][2] = 2.0*(yz - wx);
    R[2][0] = 2.0*(xz - wy);         R[2][1] = 2.0*(yz + wx);       R[2][2] = 1.0 - 2.0*(xx + yy);
}

static void matVecMul3(const double M[3][3], const double v[3], double out[3])
{
    out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
    out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}

static double vecLen3(const double v[3])
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// ── ForceController ─────────────────────────────────────────────────

void ForceController::configure(const ForceControlConfig &cfg)
{
    m_cfg = cfg;
    const double fs = 250.0;  // RSI cycle rate
    for (int i = 0; i < 6; ++i)
        m_lpf[i].configure(cfg.params.cutoffHz, fs);
    computeSensorToToolRotation();
}

void ForceController::enable(const Pose &currentCmd, const WrenchFrame &bias)
{
    m_cmd = currentCmd;
    m_bias = bias;
    // Pre-fill filter states with bias values (net force = 0)
    const double b[6] = {bias.fx, bias.fy, bias.fz, bias.mx, bias.my, bias.mz};
    for (int i = 0; i < 6; ++i)
        m_lpf[i].reset(b[i]);
    m_active = true;
}

void ForceController::disable()
{
    m_active = false;
}

double ForceController::sigmoid(double magnitude, double deadzone, double gain, double vmax)
{
    if (magnitude <= deadzone)
        return 0.0;
    const double x = gain * (magnitude - deadzone);
    return vmax * std::tanh(x);
}

double ForceController::forceVectorNorm() const
{
    return std::sqrt(m_raw.fx*m_raw.fx + m_raw.fy*m_raw.fy + m_raw.fz*m_raw.fz);
}

double ForceController::torqueVectorNorm() const
{
    return std::sqrt(m_raw.mx*m_raw.mx + m_raw.my*m_raw.my + m_raw.mz*m_raw.mz);
}

void ForceController::computeSensorToToolRotation()
{
    // Mounting gives flange_T_sensor and flange_T_tool as XYZ+ABC.
    // sensor_T_tool = inv(flange_T_sensor).rotation * flange_T_tool.rotation
    // For simplicity: build rotation from ABC angles, then compute R_st = R_fs^T × R_ft.

    double R_fs[3][3], R_ft[3][3];
    const double *s = m_cfg.mounting.flangeTSensor;  // [X,Y,Z,A,B,C]
    const double *t = m_cfg.mounting.flangeTTool;
    eulerToRotation(s[3], s[4], s[5], R_fs);
    eulerToRotation(t[3], t[4], t[5], R_ft);

    // R_st = R_fs^T × R_ft
    double tmp[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            tmp[i][j] = 0.0;
            for (int k = 0; k < 3; ++k)
                tmp[i][j] += R_fs[k][i] * R_ft[k][j];  // R_fs^T × R_ft
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m_rotSensorToTool[i][j] = tmp[i][j];
}

void ForceController::applyChannelSigns(float raw[6]) const
{
    for (int i = 0; i < 6; ++i)
        raw[i] *= float(m_cfg.sensor.channelSigns[i]);
    raw[3] *= float(m_cfg.sensor.torqueScale);
    raw[4] *= float(m_cfg.sensor.torqueScale);
    raw[5] *= float(m_cfg.sensor.torqueScale);
}

WrenchFrame ForceController::transformToBase(const float sensorValues[6],
                                              double aDeg, double bDeg, double cDeg) const
{
    // 1. Apply channel signs + torque scale
    float sv[6];
    for (int i = 0; i < 6; ++i) sv[i] = sensorValues[i];
    applyChannelSigns(sv);

    // 2. Rotate force/moment by sensor_T_tool rotation
    double Fs[3] = {double(sv[0]), double(sv[1]), double(sv[2])};
    double Ms[3] = {double(sv[3]), double(sv[4]), double(sv[5])};
    double Ft[3], Mt[3];
    matVecMul3(m_rotSensorToTool, Fs, Ft);
    matVecMul3(m_rotSensorToTool, Ms, Mt);

    // 3. Rotate from tool frame to BASE frame using current orientation
    double R_base_tool[3][3];
    eulerToRotation(aDeg, bDeg, cDeg, R_base_tool);
    double Fb[3], Mb[3];
    matVecMul3(R_base_tool, Ft, Fb);
    matVecMul3(R_base_tool, Mt, Mb);

    WrenchFrame w;
    w.fx = Fb[0]; w.fy = Fb[1]; w.fz = Fb[2];
    w.mx = Mb[0]; w.my = Mb[1]; w.mz = Mb[2];
    w.fresh = true;
    return w;
}

Pose ForceController::step(const WrenchFrame &wrench, const Pose &actualPose, double dtS)
{
    m_raw = wrench;

    if (!m_active || !wrench.fresh || dtS <= 0.0)
        return Pose{};

    // ── Butterworth filter ──
    const double raw6[6] = {wrench.fx, wrench.fy, wrench.fz, wrench.mx, wrench.my, wrench.mz};
    double filt[6];
    for (int i = 0; i < 6; ++i)
        filt[i] = m_lpf[i].step(raw6[i]);
    m_filtered = WrenchFrame{filt[0], filt[1], filt[2], filt[3], filt[4], filt[5], true};

    // ── Subtract bias ──
    const double netF[3] = {filt[0] - m_bias.fx, filt[1] - m_bias.fy, filt[2] - m_bias.fz};
    const double netM[3] = {filt[3] - m_bias.mx, filt[4] - m_bias.my, filt[5] - m_bias.mz};

    // ── Vector deadzone ──
    const double fMag = vecLen3(netF);
    const double mMag = vecLen3(netM);

    // ── Sigmoid: force magnitude → speed ──
    const double vPos = sigmoid(fMag, m_cfg.params.deadzoneForceN,
                                m_cfg.params.gainForce, m_cfg.params.vmaxPosMmS);
    const double vRot = sigmoid(mMag, m_cfg.params.deadzoneTorqueNm,
                                m_cfg.params.gainTorque, m_cfg.params.vmaxRotDegS);

    // ── Direction decomposition ──
    double vAxis[3] = {0.0, 0.0, 0.0};
    double wAxis[3] = {0.0, 0.0, 0.0};

    if (fMag > 1e-12) {
        const double invMag = 1.0 / fMag;
        vAxis[0] = vPos * netF[0] * invMag;
        vAxis[1] = vPos * netF[1] * invMag;
        vAxis[2] = vPos * netF[2] * invMag;
    }
    if (mMag > 1e-12) {
        const double invMag = 1.0 / mMag;
        wAxis[0] = vRot * netM[0] * invMag;
        wAxis[1] = vRot * netM[1] * invMag;
        wAxis[2] = vRot * netM[2] * invMag;
    }

    // ── Axis mask ──
    if (!m_cfg.axes.enX) vAxis[0] = 0.0;
    if (!m_cfg.axes.enY) vAxis[1] = 0.0;
    if (!m_cfg.axes.enZ) vAxis[2] = 0.0;
    if (!m_cfg.axes.enA) wAxis[0] = 0.0;
    if (!m_cfg.axes.enB) wAxis[1] = 0.0;
    if (!m_cfg.axes.enC) wAxis[2] = 0.0;

    // ── Position delta ──
    Pose delta;
    delta.x = vAxis[0] * dtS;
    delta.y = vAxis[1] * dtS;
    delta.z = vAxis[2] * dtS;

    // ── Attitude delta: world-frame angular velocity → Euler delta (deg) ──
    // wAxis is ω in world frame (deg/s). Rotation vector (rad) = ω * dt * π/180,
    // converted to quaternion and composed onto the current orientation, then
    // read back as ABC. This stays well-defined at B = ±90° (unlike E⁻¹).
    double rotVecRad[3];
    rotVecRad[0] = wAxis[0] * dtS * 3.141592653589793 / 180.0;
    rotVecRad[1] = wAxis[1] * dtS * 3.141592653589793 / 180.0;
    rotVecRad[2] = wAxis[2] * dtS * 3.141592653589793 / 180.0;

    using namespace poseops;
    const Quat qRot = quatFromRotVec(rotVecRad);
    // Compose: new_q = qRot ⊗ q_current — then back to ABC
    const Quat qCur = quatFromABC(actualPose.a, actualPose.b, actualPose.c);
    const Quat qNew = quatMul(qRot, qCur);
    double newA, newB, newC;
    abcFromQuat(qNew, &newA, &newB, &newC);
    delta.a = wrap180(newA - actualPose.a);
    delta.b = wrap180(newB - actualPose.b);
    delta.c = wrap180(newC - actualPose.c);

    // ── vmax hard clamp ──
    const double maxPosStep = m_cfg.params.vmaxPosMmS * dtS;
    const double maxRotStep = m_cfg.params.vmaxRotDegS * dtS;
    const double posNorm = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    if (posNorm > maxPosStep) {
        const double s = maxPosStep / posNorm;
        delta.x *= s; delta.y *= s; delta.z *= s;
    }
    const double rotNorm = std::sqrt(delta.a*delta.a + delta.b*delta.b + delta.c*delta.c);
    if (rotNorm > maxRotStep) {
        const double s = maxRotStep / rotNorm;
        delta.a *= s; delta.b *= s; delta.c *= s;
    }

    // ── Threshold quantization (mirror PoseController's wire quantum deadband) ──
    auto quantize = [](double &v) { if (std::abs(v) < 1e-4) v = 0.0; };
    quantize(delta.x); quantize(delta.y); quantize(delta.z);
    quantize(delta.a); quantize(delta.b); quantize(delta.c);

    // ── Command ledger ──
    m_cmd.x += delta.x; m_cmd.y += delta.y; m_cmd.z += delta.z;
    m_cmd.a += delta.a; m_cmd.b += delta.b; m_cmd.c += delta.c;

    return delta;
}
