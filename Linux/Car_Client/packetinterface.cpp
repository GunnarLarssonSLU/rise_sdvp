/*
    Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "packetinterface.h"
#include "utility.h"
#include <QDebug>
#include <math.h>
#include <QEventLoop>

namespace {
// CRC Table
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                                     0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                                     0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                                     0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                                     0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                                     0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                                     0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                                     0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                                     0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                                     0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                                     0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                                     0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                                     0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                                     0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                                     0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                                     0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                                     0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                                     0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                                     0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                                     0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                                     0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                                     0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                                     0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                                     0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                                     0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                                     0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                                     0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                                     0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                                     0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
}

PacketInterface::PacketInterface(QObject *parent) :
    QObject(parent)
{
    mSendBuffer = new quint8[mMaxBufferLen];
    mRxBuffer = new unsigned char[mMaxBufferLen];
    mSendBufferAck = new unsigned char[mMaxBufferLen];

    mRxState = 0;
    mRxTimer = 0;

    // Packet state
    mPayloadLength = 0;
    mRxDataPtr = 0;
    mCrcLow = 0;
    mCrcHigh = 0;
    mWaitingAck = false;

    // Debugging control - default to basic debugging
    mDebugLevel = DEBUG_VERBOSE;

    mTimer = new QTimer(this);
    mTimer->setInterval(10);
    mTimer->start();

    mHostAddress = QHostAddress("0.0.0.0");
    mHostAddress2 = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mUdpSocket = new QUdpSocket(this);
    mUdpServer = false;

    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

PacketInterface::~PacketInterface()
{
    delete[] mSendBuffer;
    delete[] mRxBuffer;
    delete[] mSendBufferAck;
}

void PacketInterface::processData(QByteArray &data)
{
    // Basic debugging that always runs to ensure we can see if processData is called
    qDebug() << "PacketInterface::processData: Called with" << data.length() << "bytes, debug level:" << mDebugLevel;
    
    if (mDebugLevel >= DEBUG_BASIC) {
        qDebug() << "PacketInterface::processData: START - Processing" << data.length() << "bytes";
        qDebug() << "PacketInterface::processData: Initial state - mRxState:" << mRxState << ", mRxDataPtr:" << mRxDataPtr << ", mPayloadLength:" << mPayloadLength;
    }
    
    unsigned char rx_data;
    const int rx_timeout = 50;

    for(int i = 0;i < data.length();i++) {
        rx_data = data[i];
        if (mDebugLevel >= DEBUG_VERBOSE) {
            qDebug() << "PacketInterface::processData: Byte" << i << "/" << data.length() << ": 0x" << QString::number(rx_data, 16).rightJustified(2, '0') << ", current state:" << mRxState;
        }

        switch (mRxState) {
        case 0:
            if (mDebugLevel >= DEBUG_DETAILED) {
                qDebug() << "PacketInterface::processData: State 0 - Looking for packet start";
            }
            if (rx_data == 2) {
                if (mDebugLevel >= DEBUG_DETAILED) {
                    qDebug() << "PacketInterface::processData: Found start byte 2, setting up for 1-byte length";
                }
                mRxState += 3;
                mRxTimer = rx_timeout;
                mRxDataPtr = 0;
                mPayloadLength = 0;
            } else if (rx_data == 3) {
                if (mDebugLevel >= DEBUG_DETAILED) {
                    qDebug() << "PacketInterface::processData: Found start byte 3, setting up for 2-byte length";
                }
                mRxState += 2;
                mRxTimer = rx_timeout;
                mRxDataPtr = 0;
                mPayloadLength = 0;
            } else if (rx_data == 4) {
                if (mDebugLevel >= DEBUG_DETAILED) {
                    qDebug() << "PacketInterface::processData: Found start byte 4, setting up for 3-byte length";
                }
                mRxState++;
                mRxTimer = rx_timeout;
                mRxDataPtr = 0;
                mPayloadLength = 0;
            } else {
                if (mDebugLevel >= DEBUG_DETAILED) {
                    qDebug() << "PacketInterface::processData: Invalid start byte 0x" << QString::number(rx_data, 16) << ", resetting state";
                }
                mRxState = 0;
            }
            break;

        case 1:
            qDebug() << "PacketInterface::processData: State 1 - Reading payload length byte 2";
            mPayloadLength |= (unsigned int)rx_data << 16;
            qDebug() << "PacketInterface::processData: mPayloadLength now:" << mPayloadLength;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 2:
            qDebug() << "PacketInterface::processData: State 2 - Reading payload length byte 1";
            mPayloadLength |= (unsigned int)rx_data << 8;
            qDebug() << "PacketInterface::processData: mPayloadLength now:" << mPayloadLength;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 3:
            qDebug() << "PacketInterface::processData: State 3 - Reading payload length byte 0";
            mPayloadLength |= (unsigned int)rx_data;
            qDebug() << "PacketInterface::processData: Final mPayloadLength:" << mPayloadLength;
            if (mPayloadLength <= mMaxBufferLen && mPayloadLength > 0) {
                qDebug() << "PacketInterface::processData: Payload length valid, moving to data collection";
                mRxState++;
                mRxTimer = rx_timeout;
            } else {
                qDebug() << "PacketInterface::processData: ERROR - Invalid payload length:" << mPayloadLength << ", resetting";
                mRxState = 0;
            }
            break;

        case 4:
            qDebug() << "PacketInterface::processData: State 4 - Collecting payload data";
            qDebug() << "PacketInterface::processData: Byte" << mRxDataPtr << "/" << mPayloadLength << ": 0x" << QString::number(rx_data, 16);
            
            // Safety check: Prevent buffer overrun and infinite loops
            if (mRxDataPtr >= mMaxBufferLen) {
                qDebug() << "PacketInterface::processData: ERROR - Buffer overrun detected!";
                qDebug() << "PacketInterface::processData: Expected" << mPayloadLength << "bytes but received too much data";
                qDebug() << "PacketInterface::processData: Resetting state to recover";
                mRxState = 0; // Reset state to recover
                mRxDataPtr = 0;
                mPayloadLength = 0;
                break;
            }
            
            // Safety check: Prevent processing packets that exceed expected length
            // We allow a small tolerance (10 bytes) for potential protocol variations
            // but anything significantly larger indicates corruption
            if (mRxDataPtr > mPayloadLength + 10) { // Small tolerance for protocol variations
                qDebug() << "PacketInterface::processData: ERROR - Payload length mismatch!";
                qDebug() << "PacketInterface::processData: Expected" << mPayloadLength << "bytes but already processed" << mRxDataPtr << "bytes";
                qDebug() << "PacketInterface::processData: Difference:" << (mRxDataPtr - mPayloadLength) << "bytes";
                qDebug() << "PacketInterface::processData: This suggests corrupted packet header or buffer overrun";
                qDebug() << "PacketInterface::processData: Resetting state to recover";
                mRxState = 0; // Reset state to recover
                mRxDataPtr = 0;
                mPayloadLength = 0;
                break;
            }
            
            mRxBuffer[mRxDataPtr++] = rx_data;
            if (mRxDataPtr == mPayloadLength) {
                qDebug() << "PacketInterface::processData: Payload collection complete, moving to CRC";
                mRxState++;
            }
            mRxTimer = rx_timeout;
            break;

        case 5:
            qDebug() << "PacketInterface::processData: State 5 - Reading CRC high byte";
            mCrcHigh = rx_data;
            qDebug() << "PacketInterface::processData: mCrcHigh:" << mCrcHigh;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 6:
            qDebug() << "PacketInterface::processData: State 6 - Reading CRC low byte";
            mCrcLow = rx_data;
            qDebug() << "PacketInterface::processData: mCrcLow:" << mCrcLow;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 7:
            qDebug() << "PacketInterface::processData: State 7 - Checking packet end and CRC";
            if (rx_data == 3) {
                qDebug() << "PacketInterface::processData: Found end byte 3";
                unsigned short calculated_crc = crc16(mRxBuffer, mPayloadLength);
                unsigned short received_crc = ((unsigned short)mCrcHigh << 8 | (unsigned short)mCrcLow);
                qDebug() << "PacketInterface::processData: Calculated CRC:" << calculated_crc << ", Received CRC:" << received_crc;
                if (calculated_crc == received_crc) {
                    qDebug() << "PacketInterface::processData: CRC valid, processing packet";
                    processPacket(mRxBuffer, mPayloadLength);
                    qDebug() << "PacketInterface::processData: processPacket returned";
                } else {
                    qDebug() << "PacketInterface::processData: ERROR - CRC mismatch!";
                }
            } else {
                qDebug() << "PacketInterface::processData: ERROR - Invalid end byte:" << rx_data << ", expected 3";
            }

            qDebug() << "PacketInterface::processData: Resetting state to 0";
            mRxState = 0;
            break;

        default:
            qDebug() << "PacketInterface::processData: ERROR - Invalid state:" << mRxState << ", resetting";
            mRxState = 0;
            break;
        }
    }
    
    qDebug() << "PacketInterface::processData: COMPLETED - Final state:" << mRxState;
}

void PacketInterface::processPacket(const unsigned char *data, int len)
{
    // Basic debugging that always runs
    qDebug() << "PacketInterface::processPacket: Called with" << len << "bytes";
    
    if (len > 0) {
        qDebug() << "PacketInterface::processPacket: First byte: 0x" << QString::number(data[0], 16);
        if (len > 1) {
            qDebug() << "PacketInterface::processPacket: Second byte: 0x" << QString::number(data[1], 16);
        }
    }
    
    if (mDebugLevel >= DEBUG_BASIC) {
        qDebug() << "PacketInterface::processPacket: Received packet, length:" << len << "bytes";
        if (mDebugLevel >= DEBUG_VERBOSE) {
            qDebug() << "PacketInterface::processPacket: First bytes:" << QByteArray((const char*)data, qMin(10, len)).toHex();
        }
    }
    
    QByteArray pkt = QByteArray((const char*)data, len);

    if (len < 2) {
        qDebug() << "PacketInterface::processPacket: ERROR - Packet too short for ID and command, length:" << len;
        if (mDebugLevel >= DEBUG_BASIC) {
            qDebug() << "PacketInterface::processPacket: ERROR - Packet too short for ID and command, length:" << len;
        }
        return;
    }

    unsigned char id = data[0];
    data++;
    len--;

    CMD_PACKET cmd = (CMD_PACKET)(quint8)data[0];
    data++;
    len--;

    qDebug() << "PacketInterface::processPacket: ID:" << id << ", Command:" << cmd << ", Remaining data:" << len << "bytes";
    
    if (mDebugLevel >= DEBUG_BASIC) {
        qDebug() << "PacketInterface::processPacket: ID:" << id << ", Command:" << cmd << ", Remaining data length:" << len;
    }

    emit packetReceived(id, cmd, pkt);

    switch (cmd) {
    case CMD_PRINTF: {
        QByteArray tmpArray((const char*)data, len);
        tmpArray.append('\0');
        emit printReceived(id, QString::fromLatin1(tmpArray));
    } break;

    case CMD_PRINTLOG: {
//        QByteArray tmpArray = QByteArray::fromRawData((const char*)data, len);
//        emit printReceived(id, QString::fromLatin1(tmpArray));
        QByteArray tmpArray((const char*)data, len);
        tmpArray.append('\0');
        emit printReceived(id, QString::fromLatin1(tmpArray));
    } break;

    case CMD_GET_ENU_REF: {
        int32_t ind = 0;
        double lat, lon, height;
        lat = utility::buffer_get_double64(data, 1e16, &ind);
        lon = utility::buffer_get_double64(data, 1e16, &ind);
        height = utility::buffer_get_double32(data, 1e3, &ind);
        emit enuRefReceived(id, lat, lon, height);
    } break;

    case CMD_AP_GET_ROUTE_PART: {
        int32_t ind = 0;
        QList<LocPoint> route;

        int routeLen = utility::buffer_get_int32(data, &ind);

        while (ind < len) {
            LocPoint p;
            p.setX(utility::buffer_get_double32_auto(data, &ind));
            p.setY(utility::buffer_get_double32_auto(data, &ind));
            p.setHeight(utility::buffer_get_double32_auto(data, &ind));
            p.setSpeed(utility::buffer_get_double32_auto(data, &ind));
            p.setTime(utility::buffer_get_int32(data, &ind));
            p.setAttributes(utility::buffer_get_uint32(data, &ind));
            route.append(p);
        }

        emit routePartReceived(id, routeLen, route);
    } break;

    case CMD_SEND_RTCM_USB: {
        QByteArray tmpArray((char*)data, len);
        emit rtcmUsbReceived(id, tmpArray);
    } break;

    case CMD_SEND_NMEA_RADIO: {
        QByteArray tmpArray((char*)data, len);
//        qDebug() << "NMEA radio len: " << len;
//        qDebug() << "NMEA radio data: " << QString(tmpArray);
        emit nmeaRadioReceived(id, tmpArray);
    } break;

    case CMD_GET_MAIN_CONFIG:
    case CMD_GET_MAIN_CONFIG_DEFAULT: {
        MAIN_CONFIG conf;

        int32_t ind = 0;
        conf.mag_use = data[ind++];
        conf.mag_comp = data[ind++];
        conf.yaw_mag_gain = utility::buffer_get_double32_auto(data, &ind);

        conf.mag_cal_cx = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_cy = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_cz = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_xx = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_xy = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_xz = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_yx = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_yy = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_yz = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_zx = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_zy = utility::buffer_get_double32_auto(data, &ind);
        conf.mag_cal_zz = utility::buffer_get_double32_auto(data, &ind);

        conf.gps_ant_x = utility::buffer_get_double32_auto(data, &ind);
        conf.gps_ant_y = utility::buffer_get_double32_auto(data, &ind);
        conf.gps_comp = data[ind++];
        conf.gps_req_rtk = data[ind++];
        conf.gps_use_rtcm_base_as_enu_ref = data[ind++];
        conf.gps_corr_gain_stat = utility::buffer_get_double32_auto(data, &ind);
        conf.gps_corr_gain_dyn = utility::buffer_get_double32_auto(data, &ind);
        conf.gps_corr_gain_yaw = utility::buffer_get_double32_auto(data, &ind);
        conf.gps_send_nmea = data[ind++];
        conf.gps_use_ubx_info = data[ind++];
        conf.gps_ubx_max_acc = utility::buffer_get_double32_auto(data, &ind);

        conf.uwb_max_corr = utility::buffer_get_double32_auto(data, &ind);

        conf.ap_repeat_routes = data[ind++];
        conf.ap_base_rad = utility::buffer_get_double32_auto(data, &ind);
        conf.ap_rad_time_ahead = utility::buffer_get_double32_auto(data, &ind);
        conf.ap_mode_time = data[ind++];
        conf.ap_max_speed = utility::buffer_get_double32_auto(data, &ind);
        conf.ap_time_add_repeat_ms = utility::buffer_get_int32(data, &ind);

        conf.log_rate_hz = utility::buffer_get_int16(data, &ind);
        conf.log_en = data[ind++];
        strcpy(conf.log_name, (const char*)(data + ind));
        ind += strlen(conf.log_name) + 1;
        conf.log_mode_ext = (LOG_EXT_MODE)data[ind++];
        conf.log_uart_baud = utility::buffer_get_uint32(data, &ind);

        // Car settings
        conf.car.yaw_use_odometry = data[ind++];
        conf.car.yaw_imu_gain = utility::buffer_get_double32_auto(data, &ind);
        conf.car.disable_motor = data[ind++];
        conf.car.simulate_motor = data[ind++];
        conf.car.clamp_imu_yaw_stationary = data[ind++];
        conf.car.use_uwb_pos = data[ind++];

        conf.car.gear_ratio = utility::buffer_get_double32_auto(data, &ind);
        conf.car.wheel_diam = utility::buffer_get_double32_auto(data, &ind);
        conf.car.motor_poles = utility::buffer_get_double32_auto(data, &ind);
        conf.car.steering_max_angle_rad = utility::buffer_get_double32_auto(data, &ind);
        conf.car.steering_center = utility::buffer_get_double32_auto(data, &ind);
        conf.car.steering_range = utility::buffer_get_double32_auto(data, &ind);
        conf.car.steering_ramp_time = utility::buffer_get_double32_auto(data, &ind);
        conf.car.axis_distance = utility::buffer_get_double32_auto(data, &ind);
//        conf.car.deadband = utility::buffer_get_double32_auto(data, &ind);

        emit configurationReceived(id, conf);
    } break;

    case CMD_LOG_LINE_USB: {
        QByteArray tmpArray = QByteArray::fromRawData((const char*)data, len);
        tmpArray[len] = '\0';
        emit logLineUsbReceived(id, QString::fromLocal8Bit(tmpArray));
    } break;

    case CMD_SET_SYSTEM_TIME: {
        int32_t ind = 0;
        qint32 sec = utility::buffer_get_int32(data, &ind);
        qint32 usec = utility::buffer_get_int32(data, &ind);
        emit systemTimeReceived(id, sec, usec);
    } break;

    case CMD_REBOOT_SYSTEM: {
        int32_t ind = 0;
        bool power_off = data[ind++];
        emit rebootSystemReceived(id, power_off);
    } break;

    case CMD_LOG_ETHERNET: {
        QByteArray tmpArray((char*)data, len);
        emit logEthernetReceived(id, tmpArray);
    } break;

        // Car commands
    case CMD_GET_STATE: {
        CAR_STATE state;
        int32_t ind = 0;
        
        DebugLevel effectiveDebugLevel = getEffectiveDebugLevel(CMD_GET_STATE);
        
        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE: Starting processing, received" << len << "bytes of data";
            if (effectiveDebugLevel >= DEBUG_VERBOSE) {
                qDebug() << "CMD_GET_STATE: First few bytes:" << QByteArray((const char*)data, qMin(20, len)).toHex();
            }
        }

        // Calculate minimum required data size:
        // 2 bytes (fw_major, fw_minor) +
        // 3*3*4 bytes (accel, gyro, mag arrays - 3 doubles each) +
        // 20*4 bytes (other double fields) +
        // 1 byte (mc_fault) +
        // 1*4 bytes (ms_today) +
        // 1*2 bytes (ap_route_left) +
        // 1*2 bytes (sensor_value)
        const int min_required_size = 2 + (3*3*4) + (20*4) + 1 + 4 + 2 + 2;
        
        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE: Minimum required size:" << min_required_size << "bytes";
            qDebug() << "CMD_GET_STATE: Data length check - received:" << len << ", required:" << min_required_size;
        }
        
        if (len < min_required_size) {
            if (effectiveDebugLevel >= DEBUG_BASIC) {
                qDebug() << "CMD_GET_STATE: ERROR - Received data too short. Expected at least" << min_required_size << "bytes, got" << len << "bytes";
                qDebug() << "CMD_GET_STATE: Aborting processing to prevent buffer overrun";
            }
            break;
        }

        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE - start processing fields" ;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 1/30 - fw_major at index" << ind;
        }
        state.fw_major = data[ind++];
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: fw_major =" << state.fw_major << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 2/30 - fw_minor at index" << ind;
        }
        state.fw_minor = data[ind++];
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: fw_minor =" << state.fw_minor << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 3/30 - roll at index" << ind;
        }
        state.roll = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: roll =" << state.roll << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 4/30 - pitch at index" << ind;
        }
        state.pitch = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: pitch =" << state.pitch << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 5/30 - yaw at index" << ind;
        }
        state.yaw = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: yaw =" << state.yaw << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing accel array at index" << ind;
        }
        state.accel[0] = utility::buffer_get_double32(data, 1e6, &ind);
        state.accel[1] = utility::buffer_get_double32(data, 1e6, &ind);
        state.accel[2] = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: accel = [" << state.accel[0] << ", " << state.accel[1] << ", " << state.accel[2] << "], index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing gyro array at index" << ind;
        }
        state.gyro[0] = utility::buffer_get_double32(data, 1e6, &ind);
        state.gyro[1] = utility::buffer_get_double32(data, 1e6, &ind);
        state.gyro[2] = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: gyro = [" << state.gyro[0] << ", " << state.gyro[1] << ", " << state.gyro[2] << "], index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing mag array at index" << ind;
        }
        state.mag[0] = utility::buffer_get_double32(data, 1e6, &ind);
        state.mag[1] = utility::buffer_get_double32(data, 1e6, &ind);
        state.mag[2] = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: mag = [" << state.mag[0] << ", " << state.mag[1] << ", " << state.mag[2] << "], index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 12/30 - px at index" << ind;
        }
        state.px = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: px =" << state.px << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 13/30 - py at index" << ind;
        }
        state.py = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: py =" << state.py << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 14/30 - speed at index" << ind;
        }
        state.speed = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: speed =" << state.speed << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 15/30 - vin at index" << ind;
        }
        state.vin = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: vin =" << state.vin << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 16/30 - temp_fet at index" << ind;
        }
        state.temp_fet = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: temp_fet =" << state.temp_fet << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 17/30 - mc_fault at index" << ind;
        }
        state.mc_fault = (mc_fault_code)data[ind++];
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: mc_fault =" << state.mc_fault << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 18/30 - px_gps at index" << ind;
        }
        state.px_gps = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: px_gps =" << state.px_gps << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 19/30 - py_gps at index" << ind;
        }
        state.py_gps = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: py_gps =" << state.py_gps << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 20/30 - ap_goal_px at index" << ind;
        }
        state.ap_goal_px = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: ap_goal_px =" << state.ap_goal_px << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 21/30 - ap_goal_py at index" << ind;
        }
        state.ap_goal_py = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: ap_goal_py =" << state.ap_goal_py << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 22/30 - ap_rad at index" << ind;
        }
        state.ap_rad = utility::buffer_get_double32(data, 1e6, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: ap_rad =" << state.ap_rad << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 23/30 - ms_today at index" << ind;
        }
        state.ms_today = utility::buffer_get_int32(data, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: ms_today =" << state.ms_today << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 24/30 - ap_route_left at index" << ind;
        }
        state.ap_route_left = utility::buffer_get_int16(data, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: ap_route_left =" << state.ap_route_left << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 25/30 - px_uwb at index" << ind;
        }
        state.px_uwb = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: px_uwb =" << state.px_uwb << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 26/30 - py_uwb at index" << ind;
        }
        state.py_uwb = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: py_uwb =" << state.py_uwb << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 27/30 - angle at index" << ind;
        }
        state.angle = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: angle =" << state.angle << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE - middle" ;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 28/30 - servo_output at index" << ind;
        }
        state.servo_output = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: servo_output =" << state.servo_output << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 29/30 - sensor_value at index" << ind;
        }
        state.sensor_value = utility::buffer_get_uint16(data, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: sensor_value =" << state.sensor_value << ", index now:" << ind;
        }
        
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: Processing field 30/30 - debug_value at index" << ind;
        }
        state.debug_value = utility::buffer_get_double32(data, 1e4, &ind);
        if (effectiveDebugLevel >= DEBUG_DETAILED) {
            qDebug() << "CMD_GET_STATE: debug_value =" << state.debug_value << ", index now:" << ind;
        }
        
        state.debug_value2 = utility::buffer_get_double32(data, 1e4, &ind);
        state.debug_value3 = utility::buffer_get_double32(data, 1e4, &ind);
        state.debug_value4 = utility::buffer_get_double32(data, 1e4, &ind);
        state.debug_value5 = utility::buffer_get_double32(data, 1e4, &ind);
        
        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE: All debug values processed, final index:" << ind;
            qDebug() << "CMD_GET_STATE: Total data length:" << len << ", bytes processed:" << ind;
        }
        
        if (ind > len) {
            if (effectiveDebugLevel >= DEBUG_BASIC) {
                qDebug() << "CMD_GET_STATE: ERROR - Processed more bytes than available! This should not happen.";
            }
        }

        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "Servo output: " << state.servo_output << ", Debug value: " << state.debug_value << "::" << state.debug_value2 << "::" << state.debug_value3 << "::" << state.debug_value4 << "::" << state.debug_value5;
            qDebug() << "CMD_GET_STATE - slut" ;
            qDebug() << "CMD_GET_STATE: Emitting stateReceived signal for ID:" << id;
        }
        emit stateReceived(id, state);
        if (effectiveDebugLevel >= DEBUG_BASIC) {
            qDebug() << "CMD_GET_STATE: Processing completed successfully";
        }
    } break;

    case CMD_VESC_FWD:
        emit vescFwdReceived(id, QByteArray::fromRawData((char*)data, len));
        break;

        // Acks
    case CMD_AP_ADD_POINTS:
        emit ackReceived(id, cmd, "CMD_AP_ADD_POINTS");
        break;
    case CMD_AP_REMOVE_LAST_POINT:
        emit ackReceived(id, cmd, "CMD_AP_REMOVE_LAST_POINT");
        break;
    case CMD_AP_CLEAR_POINTS:
        emit ackReceived(id, cmd, "CMD_AP_CLEAR_POINTS");
        break;
    case CMD_AP_SET_ACTIVE:
        emit ackReceived(id, cmd, "CMD_AP_SET_ACTIVE");
        break;
    case CMD_AP_REPLACE_ROUTE:
        emit ackReceived(id, cmd, "CMD_AP_REPLACE_ROUTE");
        break;
    case CMD_AP_SYNC_POINT:
        emit ackReceived(id, cmd, "CMD_AP_SYNC_POINT");
        break;
    case CMD_SET_MAIN_CONFIG:
        emit ackReceived(id, cmd, "CMD_SET_MAIN_CONFIG");
        break;
    case CMD_SET_POS_ACK:
        emit ackReceived(id, cmd, "CMD_SET_POS_ACK");
        break;
    case CMD_SET_ENU_REF:
        emit ackReceived(id, cmd, "CMD_SET_ENU_REF");
        break;
    case CMD_SET_YAW_OFFSET_ACK:
        emit ackReceived(id, cmd, "CMD_SET_YAW_OFFSET_ACK");
        break;
    case CMD_SET_SYSTEM_TIME_ACK:
        emit ackReceived(id, cmd, "CMD_SET_SYSTEM_TIME_ACK");
        break;
    case CMD_REBOOT_SYSTEM_ACK:
        emit ackReceived(id, cmd, "CMD_REBOOT_SYSTEM_ACK");
        break;
    case CMD_MOTE_UBX_START_BASE_ACK:
        emit ackReceived(id, cmd, "CMD_MOTE_UBX_START_BASE_ACK");
        break;
    case CMD_ADD_UWB_ANCHOR:
        emit ackReceived(id, cmd, "CMD_ADD_UWB_ANCHOR");
        break;
    case CMD_CLEAR_UWB_ANCHORS:
        emit ackReceived(id, cmd, "CMD_CLEAR_UWB_ANCHORS");
        break;

    default:
        break;
    }
}

void PacketInterface::setDebugLevel(DebugLevel level)
{
    qDebug() << "PacketInterface::setDebugLevel: Setting debug level to" << level;
    mDebugLevel = level;
}

PacketInterface::DebugLevel PacketInterface::getDebugLevel() const
{
    return mDebugLevel;
}

bool PacketInterface::isDebugEnabled() const
{
    return mDebugLevel > DEBUG_OFF;
}

void PacketInterface::setCommandDebugLevel(CMD_PACKET cmd, DebugLevel level)
{
    qDebug() << "PacketInterface::setCommandDebugLevel: Setting debug level" << level << "for command" << cmd;
    mCommandDebugLevels[cmd] = level;
}

PacketInterface::DebugLevel PacketInterface::getCommandDebugLevel(CMD_PACKET cmd) const
{
    return mCommandDebugLevels.value(cmd, DEBUG_OFF);
}

PacketInterface::DebugLevel PacketInterface::getEffectiveDebugLevel(CMD_PACKET cmd) const
{
    // Command-specific debug level takes precedence over global level
    DebugLevel cmdLevel = getCommandDebugLevel(cmd);
    if (cmdLevel > DEBUG_OFF) {
        return cmdLevel;
    }
    return mDebugLevel;
}

void PacketInterface::clearCommandDebugLevel(CMD_PACKET cmd)
{
    qDebug() << "PacketInterface::clearCommandDebugLevel: Clearing debug level for command" << cmd;
    mCommandDebugLevels.remove(cmd);
}

void PacketInterface::clearAllCommandDebugLevels()
{
    qDebug() << "PacketInterface::clearAllCommandDebugLevels: Clearing all command-specific debug levels";
    mCommandDebugLevels.clear();
}

void PacketInterface::timerSlot()
{
    if (mRxTimer) {
        mRxTimer--;
    } else {
        mRxState = 0;
    }
}

void PacketInterface::readPendingDatagrams()
{
    qDebug() << "PacketInterface::readPendingDatagrams: Called, checking for pending datagrams";
    
    if (mDebugLevel >= DEBUG_BASIC) {
        qDebug() << "PacketInterface::readPendingDatagrams: Debug level" << mDebugLevel << "active";
    }
    
    while (mUdpSocket->hasPendingDatagrams()) {
        qDebug() << "PacketInterface::readPendingDatagrams: Found pending datagram";
        
        QByteArray datagram;
        int datagramSize = mUdpSocket->pendingDatagramSize();
        qDebug() << "PacketInterface::readPendingDatagrams: Datagram size:" << datagramSize << "bytes";
        
        datagram.resize(datagramSize);
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);
        qDebug() << "PacketInterface::readPendingDatagrams: Received from" << sender.toString() << ":" << senderPort;
        
        if (datagramSize > 0) {
            qDebug() << "PacketInterface::readPendingDatagrams: First byte: 0x" << QString::number((unsigned char)datagram[0], 16);
        }
        
        if (mUdpServer) {
            qDebug() << "PacketInterface::readPendingDatagrams: UDP server mode - updating host address";
            mHostAddress = sender;
        }
        
        qDebug() << "PacketInterface::readPendingDatagrams: Calling processPacket with" << datagram.length() << "bytes";
        processPacket((unsigned char*)datagram.data(), datagram.length());
        qDebug() << "PacketInterface::readPendingDatagrams: processPacket returned";
    }
    
    qDebug() << "PacketInterface::readPendingDatagrams: Completed, no more pending datagrams";
}

unsigned short PacketInterface::crc16(const unsigned char *buf, unsigned int len)
{
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}

bool PacketInterface::sendPacket(const unsigned char *data, unsigned int len_packet)
{
    unsigned int ind = 0;
//    qDebug() << "in packetinterface::sendPacket";

    // If the IP is valid, send the packet over UDP
    if (QString::compare(mHostAddress.toString(), "0.0.0.0") != 0) {
        memcpy(mSendBufferAck + ind, data, len_packet);
        ind += len_packet;

        QByteArray toSend = QByteArray::fromRawData((const char*)mSendBufferAck, ind);
//        qDebug() << "ok and about to write";

        mUdpSocket->writeDatagram(toSend, mHostAddress, mUdpPort);
        //ros2 ...

        if (QString::compare(mHostAddress2.toString(), "0.0.0.0") != 0) {
            mUdpSocket->writeDatagram(toSend, mHostAddress2, mUdpPort);
        }

        return true;
    }

    int len_tot = len_packet;
    unsigned int data_offs = 0;

 //   qDebug() << "Length: " << len_tot;
    if (len_tot <= 255) {
        mSendBufferAck[ind++] = 2;
        mSendBufferAck[ind++] = len_tot;
        data_offs = 2;
    } else if (len_tot <= 65535) {
        mSendBufferAck[ind++] = 3;
        mSendBufferAck[ind++] = len_tot >> 8;
        mSendBufferAck[ind++] = len_tot & 0xFF;
        data_offs = 3;
    } else {
        mSendBufferAck[ind++] = 4;
        mSendBufferAck[ind++] = (len_tot >> 16) & 0xFF;
        mSendBufferAck[ind++] = (len_tot >> 8) & 0xFF;
        mSendBufferAck[ind++] = len_tot & 0xFF;
        data_offs = 4;
    }

    memcpy(mSendBufferAck + ind, data, len_packet);
    ind += len_packet;

    unsigned short crc = crc16(mSendBufferAck + data_offs, len_tot);
    mSendBufferAck[ind++] = crc >> 8;
    mSendBufferAck[ind++] = crc;
    mSendBufferAck[ind++] = 3;

    QByteArray sendData = QByteArray::fromRawData((char*)mSendBufferAck, ind);

    emit dataToSend(sendData);

    return true;
}

bool PacketInterface::sendPacket(QByteArray data)
{
    return sendPacket((const unsigned char*)data.data(), data.size());
}

/**
 * @brief PacketInterface::sendPacketAck
 * Send packet and wait for acknoledgement.
 *
 * @param data
 * The data to be sent.
 *
 * @param len_packet
 * Size of the data.
 *
 * @param retries
 * The maximum number of retries before giving up.
 *
 * @param timeoutMs
 * Time to wait before trying again.
 *
 * @return
 * True for success, false otherwise.
 */
bool PacketInterface::sendPacketAck(const unsigned char *data, unsigned int len_packet,
                                    int retries, int timeoutMs)
{
    if (mWaitingAck) {
        qDebug() << "Already waiting for packet";
        return false;
    }

    mWaitingAck = true;

    unsigned char *buffer = new unsigned char[len_packet];
    bool ok = false;
    memcpy(buffer, data, len_packet);

    for (int i = 0;i < retries;i++) {
        QEventLoop loop;
        QTimer timeoutTimer;
        timeoutTimer.setSingleShot(true);
        timeoutTimer.start(timeoutMs);
        connect(this, SIGNAL(ackReceived(quint8, CMD_PACKET, QString)), &loop, SLOT(quit()));
        connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));

        QTimer::singleShot(0, [this, buffer, len_packet]() {
            sendPacket(buffer, len_packet);
        });

        loop.exec();

        if (timeoutTimer.isActive()) {
            ok = true;
            break;
        }

        qDebug() << "Retrying to send packet...";
    }

    mWaitingAck = false;
    delete[] buffer;
    return ok;
}

bool PacketInterface::waitSignal(QObject *sender, const char *signal, int timeoutMs)
{
//    qDebug() << "in packetinterface::waitSignal";

    QEventLoop loop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);
    timeoutTimer.start(timeoutMs);
    auto conn1 = connect(sender, signal, &loop, SLOT(quit()));
    auto conn2 = connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));
    loop.exec();

    disconnect(conn1);
    disconnect(conn2);

    return timeoutTimer.isActive();
}

void PacketInterface::startUdpConnection(QHostAddress ip, int port)
{
    qDebug() << "in packetinterface::startUdpConnection";
    mHostAddress = ip;
    mUdpPort = port;
    mUdpServer = false;
    mUdpSocket->close();
    mUdpSocket->bind(QHostAddress::Any, mUdpPort + 1);
}

void PacketInterface::startUdpConnection2(QHostAddress ip)
{
    qDebug() << "in packetinterface::startUdpConnection2";
    mHostAddress2 = ip;
}

void PacketInterface::startUdpConnectionServer(int port)
{
    qDebug() << "in packetinterface::startUdpConnectionServer";
    mUdpPort = port + 1;
    mUdpServer = true;
    mUdpSocket->close();
    mUdpSocket->bind(QHostAddress::Any, mUdpPort);
}

void PacketInterface::stopUdpConnection()
{
    qDebug() << "in packetinterface::stopUdpConnection";
    mHostAddress = QHostAddress("0.0.0.0");
    mHostAddress2 = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mUdpServer = false;
    mUdpSocket->close();
}

bool PacketInterface::isUdpConnected()
{
    return QString::compare(mHostAddress.toString(), "0.0.0.0") != 0;
}

bool PacketInterface::setRoutePoints(quint8 id, QList<LocPoint> points, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_ADD_POINTS;

    for (int i = 0;i < points.size();i++) {
        LocPoint *p = &points[i];
        utility::buffer_append_double32(mSendBuffer, p->getX(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getY(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getHeight(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getSpeed(), 1e6, &send_index);
        utility::buffer_append_int32(mSendBuffer, p->getTime(), &send_index);
        utility::buffer_append_uint32(mSendBuffer, p->getAttributes(), &send_index);
    }

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::replaceRoute(quint8 id, QList<LocPoint> points, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_REPLACE_ROUTE;

    for (int i = 0;i < points.size();i++) {
        LocPoint *p = &points[i];
        utility::buffer_append_double32(mSendBuffer, p->getX(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getY(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getHeight(), 1e4, &send_index);
        utility::buffer_append_double32(mSendBuffer, p->getSpeed(), 1e6, &send_index);
        utility::buffer_append_int32(mSendBuffer, p->getTime(), &send_index);
        utility::buffer_append_uint32(mSendBuffer, p->getAttributes(), &send_index);
    }

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::removeLastRoutePoint(quint8 id, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_REMOVE_LAST_POINT;

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::clearRoute(quint8 id, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_CLEAR_POINTS;

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::setApActive(quint8 id, bool active, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_SET_ACTIVE;
    mSendBuffer[send_index++] = active ? 1 : 0;

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::setConfiguration(quint8 id, MAIN_CONFIG &conf, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_MAIN_CONFIG;

    mSendBuffer[send_index++] = conf.mag_use;
    mSendBuffer[send_index++] = conf.mag_comp;
    utility::buffer_append_double32_auto(mSendBuffer, conf.yaw_mag_gain, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_cx, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_cy, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_cz, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_xx, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_xy, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_xz, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_yx, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_yy, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_yz, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_zx, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_zy, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mag_cal_zz, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_ant_x, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_ant_y, &send_index);
    mSendBuffer[send_index++] = conf.gps_comp;
    mSendBuffer[send_index++] = conf.gps_req_rtk;
    mSendBuffer[send_index++] = conf.gps_use_rtcm_base_as_enu_ref;
    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_corr_gain_stat, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_corr_gain_dyn, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_corr_gain_yaw, &send_index);
    mSendBuffer[send_index++] = conf.gps_send_nmea;
    mSendBuffer[send_index++] = conf.gps_use_ubx_info;
    utility::buffer_append_double32_auto(mSendBuffer, conf.gps_ubx_max_acc, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.uwb_max_corr, &send_index);

    mSendBuffer[send_index++] = conf.ap_repeat_routes;
    utility::buffer_append_double32_auto(mSendBuffer, conf.ap_base_rad, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.ap_rad_time_ahead, &send_index);
    mSendBuffer[send_index++] = conf.ap_mode_time;
    utility::buffer_append_double32_auto(mSendBuffer, conf.ap_max_speed, &send_index);
    utility::buffer_append_int32(mSendBuffer, conf.ap_time_add_repeat_ms, &send_index);

    utility::buffer_append_int16(mSendBuffer, conf.log_rate_hz, &send_index);
    mSendBuffer[send_index++] = conf.log_en;
    strcpy((char*)(mSendBuffer + send_index), conf.log_name);
    send_index += strlen(conf.log_name) + 1;
    mSendBuffer[send_index++] = conf.log_mode_ext;
    utility::buffer_append_uint32(mSendBuffer, conf.log_uart_baud, &send_index);

    // Car settings
    mSendBuffer[send_index++] = conf.car.yaw_use_odometry;
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.yaw_imu_gain, &send_index);
    mSendBuffer[send_index++] = conf.car.disable_motor;
    mSendBuffer[send_index++] = conf.car.simulate_motor;
    mSendBuffer[send_index++] = conf.car.clamp_imu_yaw_stationary;
    mSendBuffer[send_index++] = conf.car.use_uwb_pos;

    utility::buffer_append_double32_auto(mSendBuffer, conf.car.gear_ratio, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.wheel_diam, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.motor_poles, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.steering_max_angle_rad, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.steering_center, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.steering_range, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.steering_ramp_time, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.car.axis_distance, &send_index);

    // Multirotor settings
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_decay_e, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_decay_l, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_max, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.map_min_x, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.map_max_x, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.map_min_y, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.map_max_y, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_gain_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_gain_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.vel_gain_d, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.tilt_gain_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.tilt_gain_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.tilt_gain_d, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.max_corr_error, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.max_tilt_error, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_roll_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_roll_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_roll_dp, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_roll_de, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pitch_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pitch_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pitch_dp, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pitch_de, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_yaw_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_yaw_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_yaw_dp, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_yaw_de, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pos_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pos_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_pos_d, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_alt_p, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_alt_i, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.ctrl_gain_alt_d, &send_index);

    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.js_gain_tilt, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, conf.mr.js_gain_yaw, &send_index);
    mSendBuffer[send_index++] = conf.mr.js_mode_rate;

    mSendBuffer[send_index++] = conf.mr.motor_fl_f;
    mSendBuffer[send_index++] = conf.mr.motor_bl_l;
    mSendBuffer[send_index++] = conf.mr.motor_fr_r;
    mSendBuffer[send_index++] = conf.mr.motor_br_b;
    mSendBuffer[send_index++] = conf.mr.motors_x;
    mSendBuffer[send_index++] = conf.mr.motors_cw;
    utility::buffer_append_uint16(mSendBuffer, conf.mr.motor_pwm_min_us, &send_index);
    utility::buffer_append_uint16(mSendBuffer, conf.mr.motor_pwm_max_us, &send_index);

    return sendPacketAck(mSendBuffer, send_index, retries, 500);
}

bool PacketInterface::setPosAck(quint8 id, double x, double y, double angle, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_POS_ACK;
    utility::buffer_append_double32(mSendBuffer, x, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, y, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, angle, 1e6, &send_index);
    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::setYawOffsetAck(quint8 id, double angle, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_YAW_OFFSET_ACK;
    utility::buffer_append_double32(mSendBuffer, angle, 1e6, &send_index);
    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::setEnuRef(quint8 id, double *llh, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_ENU_REF;
    utility::buffer_append_double64(mSendBuffer, llh[0], 1e16, &send_index);
    utility::buffer_append_double64(mSendBuffer, llh[1], 1e16, &send_index);
    utility::buffer_append_double32(mSendBuffer, llh[2], 1e3, &send_index);
    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::setSystemTime(quint8 id, qint32 sec, qint32 usec, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_SYSTEM_TIME;
    utility::buffer_append_int32(mSendBuffer, sec, &send_index);
    utility::buffer_append_int32(mSendBuffer, usec, &send_index);
    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::sendReboot(quint8 id, bool powerOff, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_REBOOT_SYSTEM;
    mSendBuffer[send_index++] = powerOff;
    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::getRoutePart(quint8 id,
                                   qint32 first,
                                   quint8 num,
                                   QList<LocPoint> &points,
                                   int &routeLen,
                                   int retries)
{
    bool appendDone = false;

    auto conn = connect(this, &PacketInterface::routePartReceived,
                        [&routeLen, &points, &appendDone](quint8 id, int len,
                        const QList<LocPoint> &route) {
        (void)id;

        if (points.isEmpty() || route.isEmpty() ||
                points.last().getDistanceTo(route.last()) > 1e-4) {
            routeLen = len;
            points.append(route);
            appendDone = true;
        }
    });

    bool res = false;
    for (int i = 0;i < retries;i++) {
        qint32 send_index = 0;
        quint8 buffer[20];
        buffer[send_index++] = id;
        buffer[send_index++] = CMD_AP_GET_ROUTE_PART;
        utility::buffer_append_int32(buffer, first, &send_index);
        buffer[send_index++] = num;

        QTimer::singleShot(0, [this, &send_index, &buffer]() {
            sendPacket(buffer, send_index);
        });

        res = waitSignal(this, SIGNAL(routePartReceived(quint8,int,QList<LocPoint>)), 200);

        if (res && appendDone) {
            break;
        }

        qDebug() << "Retrying to send packet...";
    }

    disconnect(conn);

    if (res && !appendDone) {
        qDebug() << "Route contains the same part multiple times. Make sure that it is not "
                    "corrupted.";
    }

    return res && appendDone;
}

bool PacketInterface::getRoute(quint8 id, QList<LocPoint> &points, int retries)
{
    int routeLen;
    bool ok = getRoutePart(id, points.size(), 10, points, routeLen, retries);

    while (points.size() < routeLen && ok) {
        ok = getRoutePart(id, points.size(), 10, points, routeLen, retries);
    }

    while (points.size() > routeLen) {
        points.removeLast();
    }

    return ok;
}

bool PacketInterface::setSyncPoint(quint8 id, int point, int time, int min_time_diff,
                                   bool ack, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_AP_SYNC_POINT;
    utility::buffer_append_int32(mSendBuffer, point, &send_index);
    utility::buffer_append_int32(mSendBuffer, time, &send_index);
    utility::buffer_append_int32(mSendBuffer, min_time_diff, &send_index);

    if (ack) {
        return sendPacketAck(mSendBuffer, send_index, retries);
    } else {
        return sendPacket(mSendBuffer, send_index);
    }
}

bool PacketInterface::addUwbAnchor(quint8 id, UWB_ANCHOR a, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_ADD_UWB_ANCHOR;
    utility::buffer_append_int16(mSendBuffer, a.id, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, a.px, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, a.py, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, a.height, &send_index);

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::clearUwbAnchors(quint8 id, int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_CLEAR_UWB_ANCHORS;

    return sendPacketAck(mSendBuffer, send_index, retries);
}

bool PacketInterface::sendMoteUbxBase(int mode,
                                      double pos_acc,
                                      int svin_min_dur,
                                      double svin_acc_limit,
                                      double lat,
                                      double lon,
                                      double height,
                                      int retries)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = ID_MOTE;
    mSendBuffer[send_index++] = CMD_MOTE_UBX_START_BASE;
    mSendBuffer[send_index++] = mode;
    utility::buffer_append_double64(mSendBuffer, lat, 1e16, &send_index);
    utility::buffer_append_double64(mSendBuffer, lon, 1e16, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, height, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, pos_acc, &send_index);
    utility::buffer_append_uint32(mSendBuffer, svin_min_dur, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, svin_acc_limit, &send_index);
    return sendPacketAck(mSendBuffer, send_index, retries);
}

void PacketInterface::getState(quint8 id)
{
    QByteArray packet;
    packet.append(id);
    packet.append(CMD_GET_STATE);
    sendPacket(packet);
}

void PacketInterface::getMrState(quint8 id)
{
    QByteArray packet;
    packet.append(id);
    packet.append(CMD_MR_GET_STATE);
    sendPacket(packet);
}

void PacketInterface::sendTerminalCmd(quint8 id, QString cmd)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_TERMINAL_CMD);
    packet.append(cmd.toLatin1());
    sendPacket(packet);
}

void PacketInterface::forwardVesc(quint8 id, QByteArray data)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_VESC_FWD);
    packet.append(data);
    sendPacket(packet);
}

void PacketInterface::setRcControlCurrent(quint8 id, double current, double steering)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_RC_CONTROL;
    mSendBuffer[send_index++] = RC_MODE_CURRENT;
    utility::buffer_append_double32(mSendBuffer, current, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, steering, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::setRcControlCurrentBrake(quint8 id, double current, double steering)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_RC_CONTROL;
    mSendBuffer[send_index++] = RC_MODE_CURRENT_BRAKE;
    utility::buffer_append_double32(mSendBuffer, current, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, steering, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::setRcControlDuty(quint8 id, double duty, double steering)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_RC_CONTROL;
    mSendBuffer[send_index++] = RC_MODE_DUTY;
    utility::buffer_append_double32(mSendBuffer, duty, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, steering, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::setRcControlPid(quint8 id, double speed, double steering)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_RC_CONTROL;
    mSendBuffer[send_index++] = RC_MODE_PID;
    utility::buffer_append_double32(mSendBuffer, speed, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, steering, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::setPos(quint8 id, double x, double y, double angle)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_POS;
    utility::buffer_append_double32(mSendBuffer, x, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, y, 1e4, &send_index);
    utility::buffer_append_double32(mSendBuffer, angle, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::setServoDirect(quint8 id, double value)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_SERVO_DIRECT;
    utility::buffer_append_double32(mSendBuffer, value, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::sendRtcmUsb(quint8 id, QByteArray rtcm_msg)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_SEND_RTCM_USB);
    packet.append(rtcm_msg);
    sendPacket(packet);
}

void PacketInterface::sendNmeaRadio(quint8 id, QByteArray nmea_msg)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_SEND_NMEA_RADIO);
    packet.append(nmea_msg);
    sendPacket(packet);
}

void PacketInterface::getConfiguration(quint8 id)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_GET_MAIN_CONFIG);
    sendPacket(packet);
}

void PacketInterface::getDefaultConfiguration(quint8 id)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_GET_MAIN_CONFIG_DEFAULT);
    sendPacket(packet);
}

void PacketInterface::setYawOffset(quint8 id, double angle)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_YAW_OFFSET;
    utility::buffer_append_double32(mSendBuffer, angle, 1e6, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::getEnuRef(quint8 id)
{
    QByteArray packet;
    packet.clear();
    packet.append(id);
    packet.append((char)CMD_GET_ENU_REF);
    sendPacket(packet);
}

void PacketInterface::setMsToday(quint8 id, qint32 time)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_SET_MS_TODAY;
    utility::buffer_append_int32(mSendBuffer, time, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::mrRcControl(quint8 id, double throttle, double roll, double pitch, double yaw)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_MR_RC_CONTROL;
    utility::buffer_append_double32_auto(mSendBuffer, throttle, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, roll, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, pitch, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, yaw, &send_index);
    sendPacket(mSendBuffer, send_index);
}

void PacketInterface::mrOverridePower(quint8 id, double fl_f, double bl_l, double fr_r, double br_b)
{
    qint32 send_index = 0;
    mSendBuffer[send_index++] = id;
    mSendBuffer[send_index++] = CMD_MR_OVERRIDE_POWER;
    utility::buffer_append_double32_auto(mSendBuffer, fl_f, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, bl_l, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, fr_r, &send_index);
    utility::buffer_append_double32_auto(mSendBuffer, br_b, &send_index);
    sendPacket(mSendBuffer, send_index);
}
