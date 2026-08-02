#pragma once
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPointF>
#include "core/Pose.h"

// 简化 3D TCP 姿态视图：BASE 坐标系 + 目标/实际 TCP 叠加 + 误差向量。
// 纯 OpenGL immediate mode，无 shader，~150 行。
class TcpView3D : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit TcpView3D(QWidget *parent = nullptr);

    // 更新目标与实际位姿（BASE 坐标系，位置 mm，姿态度）
    void updatePoses(const Pose &actual, const Pose &target);

    // 四元数诊断（实际姿态：qw qx qy qz；目标姿态：qw qx qy qz）
    void setQuatDiag(const Pose &actual, const Pose &target);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    void drawBaseAxes();
    void drawTcpAxes(const Pose &p, float r, float g, float b, float alpha);
    void drawErrorVector();

    double m_yaw = 30.0, m_pitch = 20.0, m_dist = 600.0;
    double m_targetX = 0, m_targetY = 0, m_targetZ = 200;
    QPointF m_lastMouse;
    bool m_dragging = false;

    Pose m_actual;
    Pose m_target;
    bool m_hasActual = false;
    bool m_hasTarget = false;
};
