#pragma once
#include <QByteArray>
#include <QString>
#include "core/Pose.h"

struct RobFrame
{
    Pose    rist;           // 实际位姿
    Pose    rsol;           // 额定位姿
    quint64 ipoc  = 0;
    quint64 delay = 0;      // KRC 统计的迟到/丢失回包数（DEF_Delay）
    bool    valid = false;
};

class RsiCodec
{
public:
    // 解析 KRC 发来的 <Rob> 报文。失败时 valid == false。
    static RobFrame parseRob(const QByteArray &datagram);

    // 生成回给 KRC 的 <Sen> 报文。ipoc 必须原样回显。
    static QByteArray buildSen(const Pose &korr, quint64 ipoc,
                               const QString &senType);
};
