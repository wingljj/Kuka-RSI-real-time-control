#include "core/RsiCodec.h"

#include <QXmlStreamReader>

#include <cmath>

namespace {

// 从元素属性读六自由度；六项缺一即失败
bool readPoseAttrs(const QXmlStreamAttributes &at, Pose *p)
{
    static const char *keys[6] = {"X", "Y", "Z", "A", "B", "C"};
    double *dst[6] = {&p->x, &p->y, &p->z, &p->a, &p->b, &p->c};

    for (int i = 0; i < 6; ++i) {
        if (!at.hasAttribute(QLatin1String(keys[i])))
            return false;
        bool ok = false;
        const double v =
            at.value(QLatin1String(keys[i])).toDouble(&ok);
        if (!ok)
            return false;
        *dst[i] = v;
    }
    return true;
}

} // namespace

RobFrame RsiCodec::parseRob(const QByteArray &datagram)
{
    RobFrame out;
    bool haveRist = false;
    bool haveIpoc = false;

    QXmlStreamReader xml(datagram);
    while (!xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement())
            continue;

        const QStringView name = xml.name();
        if (name == QLatin1String("RIst")) {
            haveRist = readPoseAttrs(xml.attributes(), &out.rist);
        } else if (name == QLatin1String("RSol")) {
            readPoseAttrs(xml.attributes(), &out.rsol);  // 可选
        } else if (name == QLatin1String("IPOC")) {
            bool ok = false;
            const quint64 v = xml.readElementText().toULongLong(&ok);
            if (ok) {
                out.ipoc = v;
                haveIpoc = true;
            }
        }
    }

    if (xml.hasError() && !(haveRist && haveIpoc))
        return out;                 // valid 仍为 false

    // 只要 RIst 与 IPOC 都已成功读到就接受该帧。真实 KRC datagram 可能带
    // 尾部填充（NUL 或空白），QXmlStreamReader 会报 "extra content at end of
    // document"；若因此一律拒绝，将是每帧都失败的全盘故障而非间歇故障。
    // 反之若 XML 在 IPOC 之前就损坏，haveRist/haveIpoc 自然为 false，仍会拒绝。
    out.valid = haveRist && haveIpoc;
    return out;
}

QByteArray RsiCodec::buildSen(const Pose &korr, quint64 ipoc,
                              const QString &senType)
{
    // 手工拼接而非 QXmlStreamWriter：报文极短且格式固定，
    // 避免在实时路径上引入额外分配与格式化开销。
    QByteArray s;
    s.reserve(256);
    s += "<Sen Type=\"";
    s += senType.toUtf8();
    s += "\">\n<RKorr";

    static const char *keys[6] = {" X=\"", " Y=\"", " Z=\"",
                                  " A=\"", " B=\"", " C=\""};
    const double vals[6] = {korr.x, korr.y, korr.z,
                            korr.a, korr.b, korr.c};
    for (int i = 0; i < 6; ++i) {
        s += keys[i];
        // 非有限值守卫：NaN 会输出 "nan"、Inf 输出 "inf"，都不是 4 位小数，
        // KRC 的 RKorr 解析不了，等同丢包并停机。而上游基于比较的限幅会
        // 传播 NaN 而非限界它，所以这道防线必须在此层——它是 wire 格式的
        // 保证者。替换为 0.0 表示"本周期无修正"，是正确的降级行为。
        const double v = std::isfinite(vals[i]) ? vals[i] : 0.0;
        s += QByteArray::number(v, 'f', 4);
        s += '"';
    }

    s += "/>\n<IPOC>";
    s += QByteArray::number(ipoc);
    s += "</IPOC>\n</Sen>";
    return s;
}
