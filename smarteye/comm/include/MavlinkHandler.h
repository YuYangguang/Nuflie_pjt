#ifndef MAVLINKHANDLER_H
#define MAVLINKHANDLER_H

#include "SerialLink.h"
#include "QGCSerialPortInfo.h"
#include <QGeoCoordinate>

class MavlinkHandler : public QObject
{
    Q_OBJECT

public:
    MavlinkHandler();

    SerialLink* m_pSerialLinkLink;

    //串口
    void ConnectToSerialPort(QString port);

    //消息解析
    void _handleHeartbeat(mavlink_message_t message);
    void _handleVfrHud(mavlink_message_t message);

    void writeBytes(const char* buffer, int len);
public slots:
    void receiveBytes(LinkInterface* link, QByteArray b);

signals:
    void Update_handleHeartbeat(QString s);
    void Update_handleHeading(QString s);
};

#endif // MAVLINKHANDLER_H
