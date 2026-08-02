#pragma once
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPointF>
#include <array>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tools/krc_simulator/rl_kinematics.h"
#include "tools/krc_simulator/MeshLoader.h"

// 实时 3D 机器人视图——OpenGL immediate mode，轨道相机，QPainter 标签覆盖。
// 有网格数据时渲染实体模型，否则 fallback 到骨架线。
// 用法：setMeshes() 设置模型几何体；每周期 updateRobot() 触发重绘。
class RobotView : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit RobotView(QWidget *parent = nullptr);

    void setMeshes(const std::vector<BodyMesh> &meshes);
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
    void drawMeshes();
    void drawLinks();
    void drawJoints();
    void drawTcp();
    void drawLabels(QPainter &p);

    // 对一组顶点应用 4x4 变换（返回 world-frame 顶点）
    static std::vector<float> transformVertices(const std::vector<float> &verts,
                                                const Eigen::Matrix4d &m);

    // 相机
    double m_yaw = 40.0, m_pitch = 25.0, m_dist = 2200.0;
    double m_targetX = 75, m_targetY = 0, m_targetZ = 800;

    QPointF m_lastMouse;
    bool m_dragging = false;

    // 数据
    rlk::Skeleton m_skel;
    double m_qDeg[6] = {0, 0, 0, 0, 0, 0};
    int m_cycle = 0, m_replies = 0, m_missed = 0;
    bool m_hasData = false;

    // 网格
    std::vector<BodyMesh> m_meshes;
    std::vector<Eigen::Matrix4d> m_homeInverse; // 预计算
    bool m_hasMeshes = false;
};
