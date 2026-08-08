#pragma once
#include "core/AppConfig.h"
#include "core/ForceFilter.h"
#include "core/Pose.h"
#include "core/Wrench.h"

// Pure-computation force control pipeline: no IO, no Qt signals.
// Safe to call from communication thread. Same style as PoseController.
class ForceController
{
public:
    ForceController() = default;

    void configure(const ForceControlConfig &cfg);

    // Enable force control. Records bias (tare), initializes command ledger and filters.
    void enable(const Pose &currentCmd, const WrenchFrame &bias);

    void disable();
    bool isActive() const { return m_active; }

    // Main pipeline. dtS = seconds since last call (wall-clock budget).
    // Returns position/attitude delta (XYZ mm, ABC deg) to send to KRC.
    Pose step(const WrenchFrame &wrench, const Pose &actualPose, double dtS);

    // Raw SRI sensor frame (float32, sensor frame) → base-frame wrench.
    // Public: RsiWorker (communication thread) feeds raw SRI samples through
    // this before the step() filter path. aDeg/bDeg/cDeg = current tool ABC.
    WrenchFrame transformToBase(const float sensorValues[6],
                                double aDeg, double bDeg, double cDeg) const;

    // Last processed wrenches for UI display
    WrenchFrame filteredWrench() const { return m_filtered; }
    WrenchFrame rawWrench() const { return m_raw; }
    WrenchFrame bias() const { return m_bias; }
    double forceVectorNorm() const;
    double torqueVectorNorm() const;

    // Command ledger (POSCORR incremental sum, mm/deg)
    Pose commandedSum() const { return m_cmd; }

    // Static: sigmoid velocity mapping
    static double sigmoid(double magnitude, double deadzone, double gain, double vmax);

private:
    ForceControlConfig m_cfg;
    bool m_active = false;

    Pose m_cmd;           // command ledger (force control mode)
    WrenchFrame m_bias;   // tare offset
    WrenchFrame m_raw;    // last raw wrench (after window-mean + coord transform)
    WrenchFrame m_filtered; // after Butterworth

    Butterworth2 m_lpf[6];  // one per channel (Fx,Fy,Fz,Mx,My,Mz)

    // Computed from mounting config: sensor_T_tool rotation (3×3)
    double m_rotSensorToTool[3][3];

    void computeSensorToToolRotation();
    void applyChannelSigns(float raw[6]) const;
};
