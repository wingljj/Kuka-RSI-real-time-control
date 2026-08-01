#include "tools/krc_simulator/RobotView.h"

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QtMath>
#include <cstdio>

RobotView::RobotView(QWidget *parent)
    : QOpenGLWidget(parent)
{
    // 鼠标追踪用于拖拽（仅在按下时启用）
    setMouseTracking(false);
    setFocusPolicy(Qt::StrongFocus);
}

void RobotView::updateRobot(const rlk::Skeleton &skel, const double qDeg[6])
{
    m_skel = skel;
    for (int i = 0; i < 6; ++i)
        m_qDeg[i] = qDeg[i];
    m_hasData = !skel.bodies.empty();
    update();  // 异步，Qt 合并到显示刷新率
}

void RobotView::setCycleInfo(int cycle, int replies, int missed)
{
    m_cycle = cycle;
    m_replies = replies;
    m_missed = missed;
}

// --------------- OpenGL ---------------

void RobotView::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.14f, 0.16f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void RobotView::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void RobotView::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 投影
    const float aspect = float(width()) / std::max(1, height());
    QMatrix4x4 proj;
    proj.perspective(45.0f, aspect, 10.0f, 50000.0f);

    // 视图（轨道相机）
    QMatrix4x4 view;
    view.translate(0, 0, -m_dist);
    view.rotate(m_pitch, 1, 0, 0);
    view.rotate(m_yaw, 0, 1, 0);
    view.translate(-m_targetX, -m_targetY, -m_targetZ);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.constData());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.constData());

    drawGrid(2400, 100);
    drawAxes(200);

    if (m_hasData) {
        drawLinks();
        drawJoints();
        drawTcp();
    }

    // QPainter 覆盖（标签、读数）
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    drawLabels(painter);
    painter.end();
}

// --------------- 绘制 ---------------

void RobotView::drawGrid(float size, float step)
{
    glColor4f(0.25f, 0.27f, 0.30f, 1.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    const float half = size * 0.5f;
    for (float x = -half; x <= half; x += step) {
        glVertex3f(x, -half, 0);
        glVertex3f(x,  half, 0);
    }
    for (float y = -half; y <= half; y += step) {
        glVertex3f(-half, y, 0);
        glVertex3f( half, y, 0);
    }
    glEnd();
}

void RobotView::drawAxes(float len)
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X = 红
    glColor3f(1, 0.2f, 0.2f);
    glVertex3f(0, 0, 0); glVertex3f(len, 0, 0);
    // Y = 绿
    glColor3f(0.2f, 1, 0.2f);
    glVertex3f(0, 0, 0); glVertex3f(0, len, 0);
    // Z = 蓝
    glColor3f(0.3f, 0.5f, 1);
    glVertex3f(0, 0, 0); glVertex3f(0, 0, len);
    glEnd();
}

void RobotView::drawLinks()
{
    const auto &b = m_skel.bodies;
    if (b.size() < 2)
        return;

    glColor3f(0.2f, 0.8f, 1.0f);  // 青色
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    for (std::size_t i = 0; i + 1 < b.size(); ++i) {
        glVertex3d(b[i].x, b[i].y, b[i].z);
        glVertex3d(b[i + 1].x, b[i + 1].y, b[i + 1].z);
    }
    glEnd();
}

void RobotView::drawJoints()
{
    const auto &b = m_skel.bodies;
    if (b.empty())
        return;

    glPointSize(8.0f);
    glBegin(GL_POINTS);
    // base 略深
    glColor3f(1.0f, 0.6f, 0.1f);  // 橙色
    glVertex3d(b[0].x, b[0].y, b[0].z);
    // 其余关节
    glColor3f(1.0f, 0.7f, 0.2f);
    for (std::size_t i = 1; i < b.size(); ++i)
        glVertex3d(b[i].x, b[i].y, b[i].z);
    glEnd();
}

void RobotView::drawTcp()
{
    const auto &t = m_skel.tcp;
    // TCP 坐标系（150 mm）
    const double len = 150.0;
    // 从四元数构造旋转矩阵（列主序）
    const double qw = t.qw, qx = t.qx, qy = t.qy, qz = t.qz;
    const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    double R[9];
    R[0] = 1 - 2 * (yy + zz); R[3] = 2 * (xy - wz);     R[6] = 2 * (xz + wy);
    R[1] = 2 * (xy + wz);     R[4] = 1 - 2 * (xx + zz); R[7] = 2 * (yz - wx);
    R[2] = 2 * (xz - wy);     R[5] = 2 * (yz + wx);     R[8] = 1 - 2 * (xx + yy);

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X (红)
    glColor3f(1, 0.2f, 0.2f);
    glVertex3d(t.x, t.y, t.z);
    glVertex3d(t.x + R[0] * len, t.y + R[1] * len, t.z + R[2] * len);
    // Y (绿)
    glColor3f(0.2f, 1, 0.2f);
    glVertex3d(t.x, t.y, t.z);
    glVertex3d(t.x + R[3] * len, t.y + R[4] * len, t.z + R[5] * len);
    // Z (蓝)
    glColor3f(0.3f, 0.5f, 1);
    glVertex3d(t.x, t.y, t.z);
    glVertex3d(t.x + R[6] * len, t.y + R[7] * len, t.z + R[8] * len);
    glEnd();

    // TCP 小球（十字）
    glPointSize(6.0f);
    glColor3f(1.0f, 0.2f, 0.2f);
    glBegin(GL_POINTS);
    glVertex3d(t.x, t.y, t.z);
    glEnd();
}

// --------------- QPainter 覆盖 ---------------

void RobotView::drawLabels(QPainter &p)
{
    p.setPen(Qt::white);
    QFont f("Consolas", 9);
    f.setStyleStrategy(QFont::PreferDefault);
    p.setFont(f);

    // 左下角读数：q1..q6 (deg) + TCP XYZ
    const auto &t = m_skel.tcp;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "A1=%6.1f  A2=%6.1f  A3=%6.1f  A4=%6.1f  A5=%6.1f  A6=%6.1f  |  "
                  "TCP  X=%7.1f  Y=%7.1f  Z=%7.1f  |  cycle=%d  rep=%d  miss=%d",
                  m_qDeg[0], m_qDeg[1], m_qDeg[2], m_qDeg[3], m_qDeg[4], m_qDeg[5],
                  m_hasData ? t.x : 0.0, m_hasData ? t.y : 0.0, m_hasData ? t.z : 0.0,
                  m_cycle, m_replies, m_missed);
    // 半透明黑底
    const QString text = QString::fromUtf8(buf);
    QRect br = p.fontMetrics().boundingRect(text).adjusted(-6, -2, 6, 2);
    br.moveBottomLeft(QPoint(8, height() - 8));
    p.fillRect(br, QColor(0, 0, 0, 160));
    p.setPen(QColor(220, 220, 220));
    p.drawText(br.adjusted(6, 2, -6, -2), Qt::AlignLeft | Qt::AlignVCenter, text);

    // 关节标签 A1..A6（投影到 body 原点）
    if (!m_hasData)
        return;

    const auto &b = m_skel.bodies;
    const std::size_t n = std::min<std::size_t>(b.size(), 6);

    // 获取当前投影 × 模型视图矩阵
    GLdouble projMat[16], mvMat[16];
    GLint vp[4];
    glGetDoublev(GL_PROJECTION_MATRIX, projMat);
    glGetDoublev(GL_MODELVIEW_MATRIX, mvMat);
    glGetIntegerv(GL_VIEWPORT, vp);

    QMatrix4x4 pm, vm;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) {
            pm(r, c) = projMat[c * 4 + r];  // GL 列主序 → QMatrix4x4 行主序
            vm(r, c) = mvMat[c * 4 + r];
        }

    QFont lf("Consolas", 8);
    lf.setStyleStrategy(QFont::PreferDefault);
    p.setFont(lf);

    for (std::size_t i = 1; i <= n; ++i) {
        const auto &bp = b[i];
        QVector4D world(bp.x, bp.y, bp.z, 1.0f);
        QVector4D clip = pm * vm * world;
        if (qFabs(clip.w()) < 1e-6f)
            continue;
        QVector3D ndc(clip.x() / clip.w(), clip.y() / clip.w(), clip.z() / clip.w());
        // NDC → 屏幕
        int sx = int((ndc.x() * 0.5f + 0.5f) * vp[2] + vp[0]);
        int sy = int((0.5f - ndc.y() * 0.5f) * vp[3] + vp[1]);
        if (sx < 0 || sx > vp[2] || sy < 0 || sy > vp[3])
            continue;
        QString label = QString("A%1").arg(i);
        int tw = p.fontMetrics().horizontalAdvance(label);
        p.setPen(QColor(0, 0, 0, 180));
        p.drawText(sx - tw / 2 + 1, sy + 12 + 1, label);
        p.setPen(QColor(255, 200, 60));
        p.drawText(sx - tw / 2, sy + 12, label);
    }
}

// --------------- 相机控制 ---------------

void RobotView::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
        m_dragging = true;
        m_lastMouse = e->position();
        setCursor(Qt::ClosedHandCursor);
    }
}

void RobotView::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_dragging)
        return;
    const QPointF delta = e->position() - m_lastMouse;
    m_lastMouse = e->position();
    m_yaw += delta.x() * 0.3;
    m_pitch += delta.y() * 0.3;
    m_pitch = qBound(-89.0, m_pitch, 89.0);
    update();
}

void RobotView::mouseReleaseEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void RobotView::wheelEvent(QWheelEvent *e)
{
    const double d = e->angleDelta().y() > 0 ? 0.9 : 1.1;
    m_dist = qBound(300.0, m_dist * d, 12000.0);
    update();
}
