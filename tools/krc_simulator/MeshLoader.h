#pragma once
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

// 单个 body 的三角网格（顶点在 world home 位姿下，单位 mm）。
struct BodyMesh {
    std::vector<float> vertices;   // x0,y0,z0, x1,y1,z1, ...（mm）
    std::vector<int> indices;      // 三角面索引，每 3 个一组
    Eigen::Matrix4d homeTransform; // 此 body 在 home 位姿下的 world 变换（用于 delta 计算）
};

// 加载 Comau Racer 7-1.4 的 link0.wrl..link6.wrl，返回 7 个 body 的网格。
// meshDir: 包含 link0.wrl..link6.wrl 的目录路径。
// homeQRad: home 位姿 q（rad），用于记录 home 变换。
std::vector<BodyMesh> loadModelMeshes(const std::string &meshDir, const double homeQRad[6]);
