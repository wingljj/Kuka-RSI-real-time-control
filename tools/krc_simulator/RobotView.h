#pragma once
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPointF>
#include <array>
#include <vector>

#include "tools/krc_simulator/rl_kinematics.h"

// 实时 3D 机器人骨架视图——OpenGL immediate mode，轨道相机，QPainter 标签覆盖。
// 用法：每周期调 updateRobot(skeleton, qDeg)，paintGL 自动合并到 ~60 fps。
class RobotView : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit RobotView(QWidget *parent = nullptr);

    // 更新机器人位姿（mm + 四元数）与关节角（度）。调用 update() 异步触发重绘。
    void updateRobot(const rlk::Skeleton &skel, const double qDeg[6]);
    void setCycleInfo(int cycle, int replies, int missed);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    void drawGrid(float size, float step);
    void drawAxes(float len);
    void drawLinks();
    void drawJoints();
    void drawTcp();
    void drawLabels(QPainter &p);

    // 相机
    double m_yaw = 40.0, m_pitch = 25.0, m_dist = 2200.0;
    double m_targetX = 75, m_targetY = 0, m_targetZ = 800;

    QPointF m_lastMouse;
    bool m_dragging = false;

    // 数据（mutex 非必须——simTick 在同一 GUI 线程）
    rlk::Skeleton m_skel;
    double m_qDeg[6] = {0, 0, 0, 0, 0, 0};
    int m_cycle = 0, m_replies = 0, m_missed = 0;
    bool m_hasData = false;
};
