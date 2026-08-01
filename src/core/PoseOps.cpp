#include "core/PoseOps.h"

#include <cmath>

namespace poseops {

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

Quat quatConj(const Quat &q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

void quatToRotMat(const Quat &q, double R[3][3])
{
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z); R[0][2] = 2*(x*z + w*y);
    R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
    R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
}
} // namespace

Quat quatMul(const Quat &a, const Quat &b)
{
    return Quat{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

Quat quatFromABC(double aDeg, double bDeg, double cDeg)
{
    const double ha = 0.5 * aDeg * kDegToRad;
    const double hb = 0.5 * bDeg * kDegToRad;
    const double hc = 0.5 * cDeg * kDegToRad;
    const Quat qz{std::cos(ha), 0, 0, std::sin(ha)};
    const Quat qy{std::cos(hb), 0, std::sin(hb), 0};
    const Quat qx{std::cos(hc), std::sin(hc), 0, 0};
    return quatMul(quatMul(qz, qy), qx);
}

void abcFromQuat(const Quat &q, double *aDeg, double *bDeg, double *cDeg)
{
    double R[3][3];
    quatToRotMat(q, R);
    const double sb = -R[2][0];
    const double cb = std::sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);
    if (cb > 1e-9) {
        *bDeg = std::atan2(sb, cb) * kRadToDeg;
        *aDeg = std::atan2(R[1][0], R[0][0]) * kRadToDeg;
        *cDeg = std::atan2(R[2][1], R[2][2]) * kRadToDeg;
    } else {
        // B = ±90°：A/C 耦合，取 C = 0 分支。R[0][1]=-sA, R[1][1]=cA → atan2(-R[0][1], R[1][1]) = A
        // gauge 说明：B=+90 时提取的 A 是 C=0-gauge 代表元 A−C；B=−90 时是 A+C。
        *bDeg = (sb > 0 ? 90.0 : -90.0);
        *aDeg = std::atan2(-R[0][1], R[1][1]) * kRadToDeg;
        *cDeg = 0.0;
    }
}

Quat quatError(const Quat &target, const Quat &actual)
{
    Quat q = quatMul(target, quatConj(actual));
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12)
        return Quat{1, 0, 0, 0};
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    return q;
}

void rotVecFromQuat(const Quat &q, double rotVec[3])
{
    const double v = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    if (v < 1e-12) { rotVec[0] = rotVec[1] = rotVec[2] = 0.0; return; }
    const double angle = 2.0 * std::atan2(v, q.w);
    rotVec[0] = q.x / v * angle;
    rotVec[1] = q.y / v * angle;
    rotVec[2] = q.z / v * angle;
}

Quat quatFromRotVec(const double rotVec[3])
{
    const double v = std::sqrt(rotVec[0]*rotVec[0] + rotVec[1]*rotVec[1] + rotVec[2]*rotVec[2]);
    if (v < 1e-12)
        return Quat{1, 0, 0, 0};
    const double angle = v;
    const double s = std::sin(0.5 * angle) / v;
    return Quat{std::cos(0.5 * angle), rotVec[0]*s, rotVec[1]*s, rotVec[2]*s};
}

Pose errorPoseDeg(const Pose &target, const Pose &actual)
{
    const Quat qA = quatFromABC(actual.a, actual.b, actual.c);
    const Quat qT = quatFromABC(target.a, target.b, target.c);
    const Quat qE = quatError(qT, qA);
    double rot[3];
    rotVecFromQuat(qE, rot);
    Pose p;
    p.x = target.x - actual.x; p.y = target.y - actual.y; p.z = target.z - actual.z;
    p.a = rot[0] * kRadToDeg; p.b = rot[1] * kRadToDeg; p.c = rot[2] * kRadToDeg;
    return p;
}

bool invEulerRate(double aDeg, double bDeg, double cDeg, double out3x3[3][3])
{
    (void)cDeg;   // E⁻¹ 不依赖 C（X 旋转不影响 yaw/pitch 速率行）
    const double a = aDeg * kDegToRad, b = bDeg * kDegToRad;
    const double sa = std::sin(a), ca = std::cos(a);
    const double sb = std::sin(b), cb = std::cos(b);
    // 安全阈值：|cosB| < 0.1（B 超出 ~±84°）即拒绝，而非只在 |cosB| < 1e-9
    // 的严格奇异点。E⁻¹ 含 1/cosB 项，B=89.99° 时 d.a/d.c 会放大到 ~687°/周期，
    // 世界系范数限幅只限世界运动、限不住发出去的欧拉增量（还会计入 m_accum）。
    // 提前回退让调用方用有界的一阶近似，才是「奇异区退化、不发散」的完整语义。
    if (std::fabs(cb) < 0.1)
        return false;   // B≈±84° 外：E⁻¹ 放大无界
    const double tb = sb / cb;
    // E⁻¹（world ZYX）：[Ȧ,Ḃ,Ċ] = E⁻¹·ω
    out3x3[0][0] =  tb*ca;  out3x3[0][1] =  tb*sa;  out3x3[0][2] = 1.0;
    out3x3[1][0] = -sa;     out3x3[1][1] =  ca;     out3x3[1][2] = 0.0;
    out3x3[2][0] =  ca/cb;  out3x3[2][1] =  sa/cb;  out3x3[2][2] = 0.0;
    return true;
}

} // namespace poseops
