#include "tools/krc_simulator/MeshLoader.h"
#include "tools/krc_simulator/rl_kinematics.h"

#include <Eigen/Dense>
#include <QFile>
#include <QString>
#include <QVector3D>
#include <QQuaternion>
#include <QRegularExpression>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <sstream>
#include <vector>
#include <string>

// 解析单个浮点数，跳过逗号
static double parseDouble(const std::string &s, std::size_t &pos)
{
    while (pos < s.size()) {
        while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == ',' || s[pos] == '\n' || s[pos] == '\r'))
            ++pos;
        if (pos < s.size() && s[pos] == '#') {
            while (pos < s.size() && s[pos] != '\n') ++pos;
            continue;
        }
        break;
    }
    std::size_t end = pos;
    while (end < s.size() && (std::isdigit((unsigned char)s[end]) || s[end] == '+' || s[end] == '-' ||
                              s[end] == '.' || s[end] == 'e' || s[end] == 'E'))
        ++end;
    if (end == pos) return 0.0;
    std::string num = s.substr(pos, end - pos);
    pos = end;
    return std::atof(num.c_str());
}

// 解析 3 个连续 double（xyz 或 translation）
static void parseVec3(const std::string &s, std::size_t &pos, double &x, double &y, double &z)
{
    x = parseDouble(s, pos);
    y = parseDouble(s, pos);
    z = parseDouble(s, pos);
}

// 解析 4 个 double（rotation: rx ry rz angle）
static void parseVec4(const std::string &s, std::size_t &pos, double &x, double &y, double &z, double &w)
{
    parseVec3(s, pos, x, y, z);
    w = parseDouble(s, pos);
}

// 从 rotation 轴角转四元数
static Eigen::Quaterniond axisAngleToQuat(double rx, double ry, double rz, double angle)
{
    double len = std::sqrt(rx*rx + ry*ry + rz*rz);
    if (len < 1e-12) return Eigen::Quaterniond::Identity();
    double ha = angle * 0.5;
    double s = std::sin(ha) / len;
    return Eigen::Quaterniond(std::cos(ha), rx * s, ry * s, rz * s);
}

// 构建 4x4 变换矩阵：translation(m) * rotation * scale
static Eigen::Matrix4d makeTransform(double tx, double ty, double tz,
                                     double rx, double ry, double rz, double angle,
                                     double sx, double sy, double sz)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,3) = tx; T(1,3) = ty; T(2,3) = tz;
    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    S(0,0) = sx; S(1,1) = sy; S(2,2) = sz;
    Eigen::Quaterniond q = axisAngleToQuat(rx, ry, rz, angle);
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R.block<3,3>(0,0) = q.toRotationMatrix();
    return T * R * S;
}

// robot.wrl places each body-local link file in the RL world home frame.
static Eigen::Matrix4d wrapperTransform(int i)
{
    switch (i) {
    case 1: return makeTransform(0.15, 0.0, 0.43, 1, 0, 0, -M_PI / 2, 1, 1, 1);
    case 2: return makeTransform(0.15, 0.0, 1.02, -0.577350, -0.577350, -0.577350,
                                 2.094395, 1, 1, 1);
    case 3: return makeTransform(0.02, 0.0, 1.02, 0, 0, 1, M_PI, 1, 1, 1);
    case 4: return makeTransform(0.02, 0.0, 1.704, 0, 0.70710677, 0.70710677,
                                 M_PI, 1, 1, 1);
    case 5: return makeTransform(0.02, 0.0, 1.704, 0, 0, 1, M_PI, 1, 1, 1);
    case 6: return makeTransform(0.02, 0.0, 1.804, 0, 0, 0, 0, 1, 1, 1);
    default: return Eigen::Matrix4d::Identity();
    }
}

// 解析单个 .wrl 文件，提取 IndexedFaceSet 顶点与三角面索引（应用变换后，单位 mm 的世界坐标）
static BodyMesh parseWrlLegacy(const std::string &path, int linkIndex)
{
    BodyMesh mesh;
    mesh.homeTransform = Eigen::Matrix4d::Identity();

    QFile f(QString::fromStdString(path));
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return mesh;

    std::string s = f.readAll().toStdString();
    f.close();

    // 找外层 Transform（DEF linkN Transform { ... }）
    // 结构：Transform { translation ... rotation ... scale ... children [ ... ] }
    // 其中 children 里可能还有内层 Transform

    // 提取所有 "translation x y z"
    struct XForm { double tx, ty, tz, rx, ry, rz, ra, sx, sy, sz; };
    std::vector<XForm> xforms;

    // 找所有 Transform { 块的位置
    std::size_t pos = 0;
    std::string key = "Transform {";
    while ((pos = s.find(key, pos)) != std::string::npos) {
        pos += key.size();
        XForm xf = {0,0,0, 0,0,1,0, 1,1,1};  // defaults

        // 在 Transform 块内（到下一个 '}' 或 "children"）搜索
        std::size_t blockEnd = s.find("children", pos);
        if (blockEnd == std::string::npos) blockEnd = s.size();
        std::string block = s.substr(pos, blockEnd - pos);

        // translation
        std::size_t tp = block.find("translation");
        if (tp != std::string::npos) {
            tp += 11;
            parseVec3(block, tp, xf.tx, xf.ty, xf.tz);
        }
        // rotation
        std::size_t rp = block.find("rotation");
        if (rp != std::string::npos) {
            rp += 8;
            parseVec4(block, rp, xf.rx, xf.ry, xf.rz, xf.ra);
        }
        // scale
        std::size_t sp = block.find("scale");
        if (sp != std::string::npos) {
            sp += 5;
            parseVec3(block, sp, xf.sx, xf.sy, xf.sz);
        }
        xforms.push_back(xf);
    }

    // 找 point 数组
    std::vector<float> pts;
    std::size_t pp = s.find("point [");
    if (pp != std::string::npos) {
        pp += 7;
        while (pp < s.size() && s[pp] != ']') {
            double x = parseDouble(s, pp);
            double y = parseDouble(s, pp);
            double z = parseDouble(s, pp);
            if (pp == 0) break;  // parseDouble didn't advance
            pts.push_back((float)x);
            pts.push_back((float)y);
            pts.push_back((float)z);
        }
    }

    // 找 coordIndex
    std::size_t cp = s.find("coordIndex [");
    if (cp != std::string::npos) {
        cp += 12;
        std::vector<int> idx;
        while (cp < s.size() && s[cp] != ']') {
            double v = parseDouble(s, cp);
            int iv = (int)v;
            if (iv < 0) {
                // -1 终止当前面（三角剖分）
                if (idx.size() >= 3) {
                    // fan triangulation: v0, v1, v2; v0, v2, v3; ...
                    for (size_t k = 1; k + 1 < idx.size(); ++k) {
                        mesh.indices.push_back(idx[0]);
                        mesh.indices.push_back(idx[k]);
                        mesh.indices.push_back(idx[k+1]);
                    }
                }
                idx.clear();
            } else {
                idx.push_back(iv);
            }
        }
        // 最后一个面（如果没有 -1 结尾）
        if (idx.size() >= 3) {
            for (size_t k = 1; k + 1 < idx.size(); ++k) {
                mesh.indices.push_back(idx[0]);
                mesh.indices.push_back(idx[k]);
                mesh.indices.push_back(idx[k+1]);
            }
        }
    }

    // VRML Transform 语义：scale 只作用于 children，不作用于自己的 translation/rotation。
    // 两层 Transform：外层 (T_out, R_out, S_out) 和内层 (T_in, R_in, 无 scale)。
    // 正确顺序：先对顶点施加 T_in * R_in，再 S_out，再 R_out * T_out。
    Eigen::Matrix4d TOut, ROut, SOut, TIn, RIn;
    TOut = ROut = SOut = TIn = RIn = Eigen::Matrix4d::Identity();

    if (xforms.size() >= 1) {
        const auto &outer = xforms[0];
        TOut(0,3) = outer.tx; TOut(1,3) = outer.ty; TOut(2,3) = outer.tz;
        SOut(0,0) = outer.sx; SOut(1,1) = outer.sy; SOut(2,2) = outer.sz;
        Eigen::Quaterniond qOut = axisAngleToQuat(outer.rx, outer.ry, outer.rz, outer.ra);
        ROut.block<3,3>(0,0) = qOut.toRotationMatrix();
    }
    if (xforms.size() >= 2) {
        const auto &inner = xforms[1];
        TIn(0,3) = inner.tx; TIn(1,3) = inner.ty; TIn(2,3) = inner.tz;
        Eigen::Quaterniond qIn = axisAngleToQuat(inner.rx, inner.ry, inner.rz, inner.ra);
        RIn.block<3,3>(0,0) = qIn.toRotationMatrix();
    }

    // 变换顺序：先内层平移+旋转 → 再 scale → 再外层旋转 → 最后外层平移
    // world_vert = TOut * ROut * SOut * TIn * RIn * local_vert
    Eigen::Matrix4d composite = TOut * ROut * SOut * TIn * RIn;

    // 应用到顶点。结果：缩放 0.001 后是米，×1000 转回毫米。
    const Eigen::Matrix4d wrapper = wrapperTransform(linkIndex);
    for (size_t i = 0; i + 2 < pts.size(); i += 3) {
        Eigen::Vector4d v(pts[i], pts[i+1], pts[i+2], 1.0);
        Eigen::Vector4d vt = wrapper * (composite * v);
        mesh.vertices.push_back((float)(vt(0) * 1000.0));
        mesh.vertices.push_back((float)(vt(1) * 1000.0));
        mesh.vertices.push_back((float)(vt(2) * 1000.0));
    }

    return mesh;
}

static BodyMesh parseWrlAccurate(const std::string &path, int linkIndex)
{
    BodyMesh mesh;
    mesh.homeTransform = Eigen::Matrix4d::Identity();
    QFile f(QString::fromStdString(path));
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return mesh;
    const std::string text = f.readAll().toStdString();
    const Eigen::Matrix4d root = wrapperTransform(linkIndex);

    auto closeBrace = [&](std::size_t open) {
        int depth = 0;
        for (std::size_t i = open; i < text.size(); ++i) {
            if (text[i] == '{') ++depth;
            else if (text[i] == '}' && --depth == 0) return i;
        }
        return text.size();
    };
    auto transformFor = [&](const std::string &h) {
        double tx=0,ty=0,tz=0,rx=0,ry=0,rz=1,a=0,sx=1,sy=1,sz=1;
        std::size_t p = h.find("translation");
        if (p != std::string::npos) { p += 11; parseVec3(h,p,tx,ty,tz); }
        p = h.find("rotation");
        if (p != std::string::npos) { p += 8; parseVec4(h,p,rx,ry,rz,a); }
        p = h.find("scale");
        if (p != std::string::npos) { p += 5; parseVec3(h,p,sx,sy,sz); }
        return makeTransform(tx,ty,tz,rx,ry,rz,a,sx,sy,sz);
    };
    std::function<void(std::size_t,std::size_t,const Eigen::Matrix4d&)> walk;
    walk = [&](std::size_t begin, std::size_t end, const Eigen::Matrix4d &parent) {
        std::size_t pos = begin;
        while (pos < end) {
            const std::size_t shape = text.find("Shape {", pos);
            const std::size_t trans = text.find("Transform {", pos);
            if (shape != std::string::npos && shape < end && (trans == std::string::npos || shape < trans)) {
                const std::size_t e = closeBrace(shape + 6);
                const std::string block = text.substr(shape, e - shape + 1);
                const std::size_t pp = block.find("point ["), cp = block.find("coordIndex [");
                if (pp != std::string::npos && cp != std::string::npos) {
                    const int base = (int)mesh.vertices.size()/3;
                    std::size_t p = pp + 7;
                    while (p < block.size() && block[p] != ']') {
                        const std::size_t before=p;
                        const double x=parseDouble(block,p), y=parseDouble(block,p), z=parseDouble(block,p);
                        if (p == before) break;
                        const Eigen::Vector4d v = parent * Eigen::Vector4d(x,y,z,1.0);
                        mesh.vertices.insert(mesh.vertices.end(), {(float)(v.x()*1000),(float)(v.y()*1000),(float)(v.z()*1000)});
                    }
                    std::vector<int> face;
                    p = cp + 12;
                    while (p < block.size() && block[p] != ']') {
                        const int v=(int)parseDouble(block,p);
                        if (v < 0) {
                            for (size_t k=1;k+1<face.size();++k) mesh.indices.insert(mesh.indices.end(),{base+face[0],base+face[k],base+face[k+1]});
                            face.clear();
                        } else face.push_back(v);
                    }
                    for (size_t k=1;k+1<face.size();++k) mesh.indices.insert(mesh.indices.end(),{base+face[0],base+face[k],base+face[k+1]});
                }
                pos=e+1; continue;
            }
            if (trans == std::string::npos || trans >= end) break;
            const std::size_t e=closeBrace(trans+9), children=text.find("children",trans);
            if (e > end || children == std::string::npos || children >= e) { pos=e+1; continue; }
            walk(children,e,parent*transformFor(text.substr(trans+10,children-(trans+10))));
            pos=e+1;
        }
    };
    walk(0,text.size(),root);
    return mesh;
}

std::vector<BodyMesh> loadModelMeshes(const std::string &meshDir, const double homeQRad[6])
{
    std::vector<BodyMesh> result;
    // 先获取 home 骨架（记录每个 body 的 home 变换）
    rlk::forward(homeQRad);
    rlk::Skeleton homeSkel = rlk::skeleton();

    for (int i = 0; i < 7; ++i) {
        char path[512];
        std::snprintf(path, sizeof(path), "%s/link%d.wrl", meshDir.c_str(), i);
        BodyMesh mesh = parseWrlAccurate(path, i);

        // 构建 body i 的 home world 变换（四元数 + 平移）
        if (i < (int)homeSkel.bodies.size()) {
            const auto &b = homeSkel.bodies[i];
            Eigen::Quaterniond q(b.qw, b.qx, b.qy, b.qz);
            mesh.homeTransform = Eigen::Matrix4d::Identity();
            mesh.homeTransform.block<3,3>(0,0) = q.toRotationMatrix();
            mesh.homeTransform(0,3) = b.x;
            mesh.homeTransform(1,3) = b.y;
            mesh.homeTransform(2,3) = b.z;
        }
        result.push_back(mesh);
    }
    return result;
}
