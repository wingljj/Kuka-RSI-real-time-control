#include <QtTest/QtTest>
#include <Eigen/Core>
#include "tools/krc_simulator/MeshLoader.h"
#include "tools/krc_simulator/rl_kinematics.h"

static std::string modelPath()
{
    const QByteArray env = qgetenv("RL_MODEL");
    return env.isEmpty() ? "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml"
                         : env.toStdString();
}

class TestMeshLoader : public QObject {
    Q_OBJECT
private slots:
    void meshHomesStayNearBodyFrames()
    {
        const QString dir = QStringLiteral("D:/QTproj/rl/rl-master/3dmodel");
        QVERIFY2(QDir(dir).exists(), qPrintable(dir));
        QVERIFY(rlk::loadModel(modelPath()));
        const double homeQ[6] = {0, 0, 0, 0, 0, 0};
        const auto meshes = loadModelMeshes(dir.toStdString(), homeQ);
        QVERIFY(meshes.size() == 7);
        rlk::forward(homeQ);
        const auto skel = rlk::skeleton();
        for (int i = 0; i < 7; ++i) {
            int bad = 0;
            for (int idx : meshes[i].indices)
                if (idx < 0 || idx >= (int)meshes[i].vertices.size()/3) ++bad;
            QVERIFY2(bad == 0, qPrintable(QString("link %1 bad indices %2").arg(i).arg(bad)));
            float maxEdge = 0.0f;
            const auto &mv = meshes[i].vertices;
            for (size_t k = 0; k + 2 < meshes[i].indices.size(); k += 3) {
                const int a = meshes[i].indices[k] * 3;
                const int b = meshes[i].indices[k + 1] * 3;
                const int cidx = meshes[i].indices[k + 2] * 3;
                auto dist = [&](int x, int y) {
                    const float dx = mv[x] - mv[y], dy = mv[x + 1] - mv[y + 1], dz = mv[x + 2] - mv[y + 2];
                    return std::sqrt(dx * dx + dy * dy + dz * dz);
                };
            maxEdge = std::max(maxEdge, std::max({dist(a,b), dist(b,cidx), dist(cidx,a)}));
            }
            QVERIFY2(maxEdge < 2000.0f, qPrintable(QString("link %1 max edge %2").arg(i).arg(maxEdge)));
            QVERIFY2(!meshes[i].vertices.empty(), qPrintable(QString("empty mesh %1").arg(i)));
            Eigen::Vector3d c = Eigen::Vector3d::Zero();
            const auto &v = meshes[i].vertices;
            for (size_t k = 0; k + 2 < v.size(); k += 3)
                c += Eigen::Vector3d(v[k], v[k + 1], v[k + 2]);
            c /= double(v.size() / 3);
            const auto &b = skel.bodies[i];
            if (i == 2) {
                QVERIFY(c.x() > 0.0 && c.x() < 300.0);
                QVERIFY(c.y() > -400.0 && c.y() < 400.0);
            }
            if (i == 6)
                QVERIFY(c.z() > 1000.0);
        }
    }
};

QTEST_APPLESS_MAIN(TestMeshLoader)
#include "test_mesh_loader.moc"
