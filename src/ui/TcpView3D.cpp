#include "ui/TcpView3D.h"
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QtMath>
#include <cmath>

TcpView3D::TcpView3D(QWidget *parent) : QOpenGLWidget(parent)
{
    setMouseTracking(false);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(200, 180);
    setStyleSheet(
        "TcpView3D { border: 1px solid #D9E0E7; border-radius: 6px; "
        "background-color: #FAFBFC; }");
}

void TcpView3D::updatePoses(const Pose &actual, const Pose &target)
{
    m_actual = actual;
    m_target = target;
    m_hasActual = true;
    m_hasTarget = true;
    update();
}

void TcpView3D::setQuatDiag(const Pose &actual, const Pose &) { (void)actual; }

// ──────────── OpenGL ────────────

void TcpView3D::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.98f, 0.985f, 0.99f, 1.0f);  // #FAFBFC
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void TcpView3D::resizeGL(int w, int h) { glViewport(0, 0, w, h); }

void TcpView3D::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const float aspect = float(width()) / std::max(1, height());
    QMatrix4x4 proj;
    proj.perspective(40.0f, aspect, 10.0f, 10000.0f);

    QMatrix4x4 view;
    view.translate(0, 0, -m_dist);
    view.rotate(m_pitch, 1, 0, 0);
    view.rotate(m_yaw, 0, 1, 0);
    view.translate(-m_targetX, -m_targetY, -m_targetZ);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.constData());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.constData());

    // 浅色网格
    glColor4f(0.85f, 0.87f, 0.89f, 1.0f);
    glLineWidth(0.5f);
    glBegin(GL_LINES);
    const float G = 200.0f;
    for (float x = -G; x <= G; x += 50.0f) {
        glVertex3f(x, -G, 0); glVertex3f(x, G, 0);
    }
    for (float y = -G; y <= G; y += 50.0f) {
        glVertex3f(-G, y, 0); glVertex3f(G, y, 0);
    }
    glEnd();

    drawBaseAxes();

    if (m_hasTarget) drawTcpAxes(m_target, 0.5f, 0.2f, 0.5f, 0.9f);  // 蓝色半透明
    if (m_hasActual) drawTcpAxes(m_actual, 1.0f, 0.55f, 0.1f, 1.0f); // 橙色实心

    if (m_hasActual && m_hasTarget) drawErrorVector();

    // QPainter 覆盖（浅色）
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QColor("#1F2937"));
    QFont f("Segoe UI", 8);
    painter.setFont(f);

    // 标题栏
    painter.fillRect(0, 0, width(), 22, QColor(255, 255, 255, 220));
    painter.setPen(QColor("#1F2937"));
    QFont tf("Segoe UI", 9);
    tf.setBold(true);
    painter.setFont(tf);
    painter.drawText(8, 16, "3D 位姿视图");

    // 图例
    painter.setFont(f);
    painter.setPen(QColor("#2563EB"));
    painter.drawText(width() - 180, 16, "目标 TCP");
    painter.setPen(QColor("#D97706"));
    painter.drawText(width() - 120, 16, "实际 TCP");
    painter.setPen(QColor("#DC2626"));
    painter.drawText(width() - 60, 16, "误差");

    // 四元数读数（左下角）
    if (m_hasActual) {
        painter.fillRect(2, height() - 36, 220, 34, QColor(255, 255, 255, 200));
        painter.setPen(QColor("#64748B"));
        painter.setFont(QFont("Consolas", 7));
        const double ax = qDegreesToRadians(m_actual.a);
        const double ay = qDegreesToRadians(m_actual.b);
        const double az = qDegreesToRadians(m_actual.c);
        const double c1 = std::cos(ax*0.5), s1 = std::sin(ax*0.5);
        const double c2 = std::cos(ay*0.5), s2 = std::sin(ay*0.5);
        const double c3 = std::cos(az*0.5), s3 = std::sin(az*0.5);
        const double qw = c1*c2*c3 + s1*s2*s3;
        const double qx = s1*c2*c3 - c1*s2*s3;
        const double qy = c1*s2*c3 + s1*c2*s3;
        const double qz = c1*c2*s3 - s1*s2*c3;
        painter.drawText(6, height() - 18,
                         QStringLiteral("qw %1  qx %2  qy %3  qz %4")
                             .arg(qw,0,'f',3).arg(qx,0,'f',3)
                             .arg(qy,0,'f',3).arg(qz,0,'f',3));
    }
    painter.end();
}

// ──────────── 绘制 ────────────

void TcpView3D::drawBaseAxes()
{
    const float L = 150.0f;
    glLineWidth(2.5f);
    glBegin(GL_LINES);
    glColor3f(0.86f, 0.08f, 0.24f);  glVertex3f(0, 0, 0); glVertex3f(L, 0, 0);  // X
    glColor3f(0.13f, 0.64f, 0.29f);  glVertex3f(0, 0, 0); glVertex3f(0, L, 0);  // Y
    glColor3f(0.22f, 0.39f, 0.88f);  glVertex3f(0, 0, 0); glVertex3f(0, 0, L);  // Z
    glEnd();
}

void TcpView3D::drawTcpAxes(const Pose &p, float r, float g, float b, float alpha)
{
    const float L = 70.0f;
    const double ax = qDegreesToRadians(p.a);
    const double ay = qDegreesToRadians(p.b);
    const double az = qDegreesToRadians(p.c);
    const double ca = std::cos(ax), sa = std::sin(ax);
    const double cb = std::cos(ay), sb = std::sin(ay);
    const double cg = std::cos(az), sg = std::sin(az);
    double R[9];
    R[0]=cb*cg; R[3]=sa*sb*cg-ca*sg; R[6]=ca*sb*cg+sa*sg;
    R[1]=cb*sg; R[4]=sa*sb*sg+ca*cg; R[7]=ca*sb*sg-sa*cg;
    R[2]=-sb;   R[5]=sa*cb;           R[8]=ca*cb;

    glLineWidth(alpha < 0.7f ? 1.5f : 2.5f);
    glBegin(GL_LINES);
    glColor4f(r, g, b, alpha);
    glVertex3d(p.x, p.y, p.z); glVertex3d(p.x+R[0]*L, p.y+R[1]*L, p.z+R[2]*L);
    glVertex3d(p.x, p.y, p.z); glVertex3d(p.x+R[3]*L, p.y+R[4]*L, p.z+R[5]*L);
    glVertex3d(p.x, p.y, p.z); glVertex3d(p.x+R[6]*L, p.y+R[7]*L, p.z+R[8]*L);
    glEnd();

    glPointSize(alpha < 0.7f ? 4.0f : 5.0f);
    glBegin(GL_POINTS);
    glColor4f(r, g, b, alpha);
    glVertex3d(p.x, p.y, p.z);
    glEnd();
}

void TcpView3D::drawErrorVector()
{
    const double dx = m_actual.x - m_target.x;
    const double dy = m_actual.y - m_target.y;
    const double dz = m_actual.z - m_target.z;
    const double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 0.5) return;

    const int segs = int(len / 15.0) + 1;
    glLineWidth(1.5f);
    glColor4f(0.86f, 0.15f, 0.15f, 0.85f);
    glBegin(GL_LINES);
    for (int i = 0; i < segs; i += 2) {
        const double t0 = double(i)/segs, t1 = double(i+1)/segs;
        glVertex3d(m_target.x+dx*t0, m_target.y+dy*t0, m_target.z+dz*t0);
        glVertex3d(m_target.x+dx*t1, m_target.y+dy*t1, m_target.z+dz*t1);
    }
    glEnd();
}

// ──────────── 相机 ────────────

void TcpView3D::mousePressEvent(QMouseEvent *e) {
    if (e->button()==Qt::LeftButton) { m_dragging=true; m_lastMouse=e->position(); setCursor(Qt::ClosedHandCursor); }
}
void TcpView3D::mouseMoveEvent(QMouseEvent *e) {
    if (!m_dragging) return;
    QPointF d=e->position()-m_lastMouse; m_lastMouse=e->position();
    m_yaw+=d.x()*0.3; m_pitch+=d.y()*0.3; m_pitch=qBound(-89.0,m_pitch,89.0); update();
}
void TcpView3D::mouseReleaseEvent(QMouseEvent *e) {
    if (e->button()==Qt::LeftButton) { m_dragging=false; setCursor(Qt::ArrowCursor); }
}
void TcpView3D::wheelEvent(QWheelEvent *e) {
    m_dist=qBound(100.0,m_dist*(e->angleDelta().y()>0?0.9:1.1),4000.0); update();
}
