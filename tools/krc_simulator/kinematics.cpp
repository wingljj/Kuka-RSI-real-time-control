#include "tools/krc_simulator/kinematics.h"

#include <algorithm>
#include <cmath>

namespace kr210 {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

const std::array<DHRow, 6> kDh{{
    { -90.0 * kDegToRad,  350.0,  675.0 },
    {   0.0 * kDegToRad, 1150.0,    0.0 },
    { -90.0 * kDegToRad,   41.0,    0.0 },
    {  90.0 * kDegToRad,    0.0, 1200.0 },
    {  90.0 * kDegToRad,    0.0,    0.0 },
    {   0.0 * kDegToRad,    0.0,  240.0 },
}};

const JointLimits &limits()
{
    static const JointLimits kL{
        {-185 * kDegToRad, -135 * kDegToRad, -175 * kDegToRad,
         -350 * kDegToRad, -122.5 * kDegToRad, -350 * kDegToRad},
        { 185 * kDegToRad,  155 * kDegToRad,   75 * kDegToRad,
          350 * kDegToRad,  122.5 * kDegToRad,  350 * kDegToRad},
    };
    return kL;
}

// 旋转矩阵 → KUKA A/B/C（ZYX: R = Rz(A)·Ry(B)·Rx(C)）。奇异取 C=0。
void eulerFromR(const double R[3][3], double *a, double *b, double *c)
{
    const double r00 = R[0][0], r10 = R[1][0], r20 = R[2][0];
    const double r21 = R[2][1], r22 = R[2][2];
    const double sb = -r20;
    const double cb = std::sqrt(r00 * r00 + r10 * r10);
    if (cb > 1e-9) {
        *b = std::atan2(sb, cb);
        *a = std::atan2(r10, r00);
        *c = std::atan2(r21, r22);
    } else {
        // B = ±90°：A/C 耦合，取 C = 0 分支。
        *b = (sb > 0) ? 0.5 * 3.14159265358979323846 : -0.5 * 3.14159265358979323846;
        *a = std::atan2(R[0][1], R[1][1]);
        *c = 0.0;
    }
}

Pose forward(const double qRad[6])
{
    double T[4][4] = {
        {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1},
    };
    for (int i = 0; i < 6; ++i) {
        const double th = qRad[i];
        const double ca = std::cos(kDh[i].alpha);
        const double sa = std::sin(kDh[i].alpha);
        const double ct = std::cos(th);
        const double st = std::sin(th);
        const double Ti[4][4] = {
            { ct, -st * ca,  st * sa, kDh[i].a * ct },
            { st,  ct * ca, -ct * sa, kDh[i].a * st },
            { 0,   sa,       ca,      kDh[i].d      },
            { 0,   0,        0,       1             },
        };
        double R[4][4];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                R[r][c] = 0.0;
                for (int k = 0; k < 4; ++k)
                    R[r][c] += T[r][k] * Ti[k][c];
            }
        std::copy(&R[0][0], &R[0][0] + 16, &T[0][0]);
    }
    Pose p;
    p.x = T[0][3]; p.y = T[1][3]; p.z = T[2][3];
    double a = 0, b = 0, c = 0;
    const double Rm[3][3] = {
        {T[0][0], T[0][1], T[0][2]},
        {T[1][0], T[1][1], T[1][2]},
        {T[2][0], T[2][1], T[2][2]},
    };
    eulerFromR(Rm, &a, &b, &c);
    p.a = a * kRadToDeg;
    p.b = b * kRadToDeg;
    p.c = c * kRadToDeg;
    return p;
}

// 前 i 个 DH 变换的 T0i 原点与 z 轴（用于雅可比）。
void chainFrames(const double qRad[6], double P[7][3], double Z[7][3])
{
    P[0][0] = P[0][1] = P[0][2] = 0.0;
    Z[0][0] = Z[0][1] = 0.0; Z[0][2] = 1.0;
    double T[4][4] = {
        {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1},
    };
    for (int i = 0; i < 6; ++i) {
        const double th = qRad[i];
        const double ca = std::cos(kDh[i].alpha);
        const double sa = std::sin(kDh[i].alpha);
        const double ct = std::cos(th);
        const double st = std::sin(th);
        const double Ti[4][4] = {
            { ct, -st * ca,  st * sa, kDh[i].a * ct },
            { st,  ct * ca, -ct * sa, kDh[i].a * st },
            { 0,   sa,       ca,      kDh[i].d      },
            { 0,   0,        0,       1             },
        };
        double R[4][4];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                R[r][c] = 0.0;
                for (int k = 0; k < 4; ++k)
                    R[r][c] += T[r][k] * Ti[k][c];
            }
        std::copy(&R[0][0], &R[0][0] + 16, &T[0][0]);
        P[i + 1][0] = T[0][3]; P[i + 1][1] = T[1][3]; P[i + 1][2] = T[2][3];
        Z[i + 1][0] = T[0][2]; Z[i + 1][1] = T[1][2]; Z[i + 1][2] = T[2][2];
    }
}

void jacobian(const double qRad[6], double J[6][6])
{
    double P[7][3], Z[7][3];
    chainFrames(qRad, P, Z);
    const double *pe = P[6];
    for (int i = 0; i < 6; ++i) {
        const double *z = Z[i];       // 轴 i+1 的旋转轴（frame i）
        const double *p = P[i];       // frame i 原点
        // Jv[i] = z × (pe - p)
        J[0][i] = z[1] * (pe[2] - p[2]) - z[2] * (pe[1] - p[1]);
        J[1][i] = z[2] * (pe[0] - p[0]) - z[0] * (pe[2] - p[2]);
        J[2][i] = z[0] * (pe[1] - p[1]) - z[1] * (pe[0] - p[0]);
        // Jw[i] = z
        J[3][i] = z[0];
        J[4][i] = z[1];
        J[5][i] = z[2];
    }
}

// 6×6 矩阵求逆（高斯-约当）。返回 false 若奇异。
bool inv6(const double A[6][6], double out[6][6])
{
    double M[6][12];
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c)
            M[r][c] = A[r][c];
        for (int c = 0; c < 6; ++c)
            M[r][6 + c] = (r == c) ? 1.0 : 0.0;
    }
    for (int col = 0; col < 6; ++col) {
        int piv = col;
        for (int r = col + 1; r < 6; ++r)
            if (std::fabs(M[r][col]) > std::fabs(M[piv][col]))
                piv = r;
        if (std::fabs(M[piv][col]) < 1e-12)
            return false;
        if (piv != col)
            for (int c = 0; c < 12; ++c)
                std::swap(M[piv][c], M[col][c]);
        const double d = M[col][col];
        for (int c = 0; c < 12; ++c)
            M[col][c] /= d;
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const double f = M[r][col];
            for (int c = 0; c < 12; ++c)
                M[r][c] -= f * M[col][c];
        }
    }
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            out[r][c] = M[r][6 + c];
    return true;
}

bool solveDelta(const double qRad[6], const Pose &dxDeg, double dqRad[6])
{
    double J[6][6];
    jacobian(qRad, J);
    // 阻尼伪逆：J⁺ = Jᵀ (J Jᵀ + λ I)⁻¹
    double JJt[6][6];
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            double s = 0.0;
            for (int k = 0; k < 6; ++k)
                s += J[r][k] * J[c][k];
            JJt[r][c] = s;
        }
    // 阻尼：小固定值。非奇异位形下伪逆≈真逆（往返精确）；近奇异时由 λ 限制
    // 增益，幅值由关节限位 clamp 兜底。按 JJᵀ 最大特征缩放会使 λ≈1.6（本机
    // 连杆臂~1250mm），对腕部模式（σ_min~0.3–1）衰减 30–70%——过强。
    constexpr double lam = 1e-3;
    for (int r = 0; r < 6; ++r)
        JJt[r][r] += lam;

    double JJtInv[6][6];
    if (!inv6(JJt, JJtInv))
        return false;

    // Δx（度→rad 姿态）
    const double dx[6] = {
        dxDeg.x, dxDeg.y, dxDeg.z,
        dxDeg.a * kDegToRad, dxDeg.b * kDegToRad, dxDeg.c * kDegToRad,
    };
    // Δq = Jᵀ (J Jᵀ)⁻¹ Δx
    double t[6] = {0, 0, 0, 0, 0, 0};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            t[r] += JJtInv[r][c] * dx[c];
    for (int i = 0; i < 6; ++i) {
        double s = 0.0;
        for (int k = 0; k < 6; ++k)
            s += J[k][i] * t[k];
        dqRad[i] = s;
    }
    return true;
}

void limitCartDelta(Pose *dx, const Pose &prevDx,
                    double maxVelPos, double maxVelRot,
                    double maxAccelPos, double maxAccelRot,
                    double cycleS)
{
    if (dx == nullptr || cycleS <= 0.0)
        return;
    auto clampTo = [](double v, double lim) {
        return lim <= 0.0 ? v : std::clamp(v, -lim, lim);
    };
    // 速度限制
    dx->x = clampTo(dx->x, maxVelPos * cycleS);
    dx->y = clampTo(dx->y, maxVelPos * cycleS);
    dx->z = clampTo(dx->z, maxVelPos * cycleS);
    dx->a = clampTo(dx->a, maxVelRot * cycleS);
    dx->b = clampTo(dx->b, maxVelRot * cycleS);
    dx->c = clampTo(dx->c, maxVelRot * cycleS);
    // 加速度限制（基于上一周期增量，允许反向）
    const double accPos = maxAccelPos * cycleS * cycleS;
    const double accRot = maxAccelRot * cycleS * cycleS;
    dx->x = clampTo(dx->x, prevDx.x + accPos);
    dx->x = clampTo(dx->x, prevDx.x - accPos);
    dx->y = clampTo(dx->y, prevDx.y + accPos);
    dx->y = clampTo(dx->y, prevDx.y - accPos);
    dx->z = clampTo(dx->z, prevDx.z + accPos);
    dx->z = clampTo(dx->z, prevDx.z - accPos);
    dx->a = clampTo(dx->a, prevDx.a + accRot);
    dx->a = clampTo(dx->a, prevDx.a - accRot);
    dx->b = clampTo(dx->b, prevDx.b + accRot);
    dx->b = clampTo(dx->b, prevDx.b - accRot);
    dx->c = clampTo(dx->c, prevDx.c + accRot);
    dx->c = clampTo(dx->c, prevDx.c - accRot);
}

} // namespace kr210
