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
        } else if (name == QLatin1String("Delay")) {
            // DEF_Delay 是 KRC 自己统计的迟到/丢失回包数，唯一能让主机看见
            // "KRC 认为我丢包了"的量。诊断字段，解析失败不拒绝整帧。
            const QStringView d = xml.attributes().value(QLatin1String("D"));
            bool ok = false;
            const quint64 v = d.toULongLong(&ok);
            if (ok)
                out.delay = v;
        } else if (name == QLatin1String("IPOC")) {
            const QString t = xml.readElementText();
            // 必须在此检查 reader 状态。实测 (Qt 6.5.3)：若部分数字后还跟着
            // 标记（<IPOC>5551< 或 <IPOC>5551</IPOC），readElementText() 会
            // 返回“已累积的部分数字” 5551 并同时置错误位；而纯粹截断在数字
            // 末尾（<IPOC>5551）则返回空串。前一种若不检查就会回显 5551，
            // 真实值可能是 5551234——IPOC 字节精确是硬契约，回错等同丢包。
            // 现实触发场景是接收缓冲区过小导致 readDatagram 静默截断，
            // 而 IPOC 恰位于 RSI 报文末尾。
            if (!xml.hasError()) {
                bool ok = false;
                const quint64 v = t.toULongLong(&ok);
                if (ok) {
                    out.ipoc = v;
                    haveIpoc = true;
                }
            }
        }
    }

    // 尾部填充容忍：真实 KRC datagram 可能带尾部 NUL 或空白，
    // QXmlStreamReader 会就此报错；若因任何 reader 错误一律拒绝，将是每帧
    // 都失败的全盘故障而非间歇故障。此处只依据“两个必需元素是否都完整读到”
    // 判定——IPOC 的截断已在上面的 hasError 守卫处挡掉，RIst 的属性在
    // StartElement 时就已完整解析（否则不会 emit），故二者均可信。
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
