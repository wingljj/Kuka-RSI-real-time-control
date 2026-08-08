#include "core/ForceFilter.h"
#include <cmath>

void Butterworth2::configure(double fc, double fs)
{
    // 不信任配置：fc 超出 (0, fs/2) 时预畸变系数 c = 1/tan(ω0·T/2) 会变成
    // ≤ 0，极点移出单位圆（如 fc=fs/2 恰好在 z=-1 上，fc>fs/2 直接发散）。
    // 回退为直通（b0=1），与未配置的默认状态语义一致，绝不出不稳定系数。
    if (!(fs > 0.0 && fc > 0.0 && fc < fs * 0.5)) {
        b0 = 1.0; b1 = 0.0; b2 = 0.0;
        a1 = 0.0; a2 = 0.0;
        x1 = x2 = y1 = y2 = 0.0;
        return;
    }

    // Bilinear transform for 2nd-order Butterworth: H(s)=1/(s²+√2·s+1)
    const double w0 = 2.0 * 3.141592653589793 * fc;
    const double T  = 1.0 / fs;
    const double c  = 1.0 / std::tan(w0 * T / 2.0);

    const double c2 = c * c;
    const double sqrt2c = 1.4142135623730951 * c;
    const double denom = c2 + sqrt2c + 1.0;

    b0 = 1.0 / denom;
    b1 = 2.0 / denom;
    b2 = 1.0 / denom;
    a1 = 2.0 * (1.0 - c2) / denom;
    a2 = (c2 - sqrt2c + 1.0) / denom;

    x1 = x2 = y1 = y2 = 0.0;
}

double Butterworth2::step(double x)
{
    // Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x;
    y2 = y1; y1 = y;
    return y;
}

void Butterworth2::reset(double initialValue)
{
    // Pre-fill state so that if x[n]=x[n-1]=x[n-2]=initialValue,
    // then y[n]=y[n-1]=y[n-2]=initialValue (steady-state).
    // 直通时 b0=1 且其余系数为 0，同样的赋值也成立，无需分支。
    x1 = x2 = initialValue;
    y1 = y2 = initialValue;
}
