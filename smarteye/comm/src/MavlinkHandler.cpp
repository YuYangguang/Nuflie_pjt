#include "MavlinkHandler.h"

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

MavlinkHandler::MavlinkHandler():
    QObject()
{

};

void MavlinkHandler::_handleHeartbeat(mavlink_message_t message)
{
    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    static int c=0;
    c++;
    QString s;
    s=QString::number(c);
    emit Update_handleHeartbeat(s);
}

void MavlinkHandler::_handleVfrHud(mavlink_message_t message)
{
    mavlink_vfr_hud_t vfrHud;
    mavlink_msg_vfr_hud_decode(&message, &vfrHud);

    QString s;
    s=QString::number(vfrHud.heading);

    emit Update_handleHeading(s);
}

void MavlinkHandler::receiveBytes(LinkInterface* link, QByteArray b)
{
    mavlink_message_t message;
    mavlink_status_t status;

    for (int position = 0; position < b.size(); position++)
    {
        if (mavlink_parse_char(link->mavlinkChannel(), (uint8_t)(b[position]), &message, &status) == 1)
        {
            if (!link->decodedFirstMavlinkPacket())
            {
                link->setDecodedFirstMavlinkPacket(true);
            }

            switch (message.msgid)
            {
                 case MAVLINK_MSG_ID_HEARTBEAT:
                    _handleHeartbeat(message);
                    break;
                case MAVLINK_MSG_ID_VFR_HUD:
                    _handleVfrHud(message);
                    break;
            }
        }
    }
}

void MavlinkHandler::writeBytes(const char* buffer, int len)
{
    m_pSerialLinkLink->writeBytesSafe(buffer, len);
}


/////////////////////////////////////////////////////////////////////////
//串口
//摘自LinkManager.cc - void LinkManager::_updateAutoConnectLinks(void)
/////////////////////////////////////////////////////////////////////////

void MavlinkHandler::ConnectToSerialPort(QString port)
{
    SerialConfiguration* pSerialConfig=new SerialConfiguration("Serial Link");
    pSerialConfig->setBaud(57600);
    pSerialConfig->setPortName(port);

    m_pSerialLinkLink = new SerialLink(SharedLinkConfigurationPointer(pSerialConfig));
    m_pSerialLinkLink->_setMavlinkChannel(1);

    bool ret = m_pSerialLinkLink->_connect();

    if (!ret) return ;

    connect(m_pSerialLinkLink, &LinkInterface::bytesReceived,        this,   &MavlinkHandler::receiveBytes);
}
