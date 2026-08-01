#include "core/AppConfig.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

namespace {

void readDouble(const QJsonObject &o, const char *key, double *dst)
{
    if (o.contains(key) && o.value(key).isDouble())
        *dst = o.value(key).toDouble();
}

void readInt(const QJsonObject &o, const char *key, int *dst)
{
    if (o.contains(key) && o.value(key).isDouble())
        *dst = o.value(key).toInt();
}

void readString(const QJsonObject &o, const char *key, QString *dst)
{
    if (o.contains(key) && o.value(key).isString())
        *dst = o.value(key).toString();
}

} // namespace

bool AppConfig::loadFromFile(const QString &path, AppConfig *out,
                             QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error)
            *error = QStringLiteral("cannot open %1: %2")
                         .arg(path, f.errorString());
        return false;
    }

    QJsonParseError pe{};
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        if (error)
            *error = QStringLiteral("JSON parse error at offset %1: %2")
                         .arg(pe.offset)
                         .arg(pe.errorString());
        return false;
    }
    if (!doc.isObject()) {
        if (error)
            *error = QStringLiteral("root is not a JSON object");
        return false;
    }

    const QJsonObject root = doc.object();

    const QJsonObject net = root.value("network").toObject();
    readString(net, "listen_ip", &out->listenIp);
    if (net.value("listen_port").isDouble()) {
        const int p = net.value("listen_port").toInt(-1);
        if (p > 0 && p <= 65535)
            out->listenPort = quint16(p);
    }
    readInt(net, "rx_buffer_bytes", &out->rxBufferBytes);

    const QJsonObject rsi = root.value("rsi").toObject();
    readDouble(rsi, "cycle_ms", &out->cycleMs);
    readDouble(rsi, "session_gap_ms", &out->sessionGapMs);
    readString(rsi, "sen_type", &out->senType);
    readInt(rsi, "watchdog_miss_limit", &out->watchdogMissLimit);
    readDouble(rsi, "target_trajectory_ms", &out->targetTrajectoryMs);
    readInt(rsi, "krc_timeout_cycles", &out->krcTimeoutCycles);
    readDouble(rsi, "krc_poscorr_limit_pos_mm", &out->krcPoscorrLimitPosMm);
    readDouble(rsi, "krc_poscorr_limit_rot_deg", &out->krcPoscorrLimitRotDeg);

    const QJsonObject ctl = root.value("control").toObject();
    readDouble(ctl, "kp_pos", &out->kpPos);
    readDouble(ctl, "kp_rot", &out->kpRot);
    readDouble(ctl, "vmax_pos_mm_s", &out->vmaxPosMmS);
    readDouble(ctl, "vmax_rot_deg_s", &out->vmaxRotDegS);
    readDouble(ctl, "accum_limit_pos_mm", &out->accumLimitPosMm);
    readDouble(ctl, "accum_limit_rot_deg", &out->accumLimitRotDeg);

    const QJsonObject ui = root.value("ui").toObject();
    readInt(ui, "refresh_ms", &out->refreshMs);
    readInt(ui, "chart_window_s", &out->chartWindowS);

    if (error)
        error->clear();
    return true;
}
