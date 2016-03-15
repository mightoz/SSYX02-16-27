#!/usr/bin/env python
PKG = 'numpy'

import socket
import numpy as np

def swapbytes16(x):
    """

    :param x: A 16-bit unsigned integer or array of integers.
    :return: The reverse order of the bytes of the given number or array.
    """
    return ((x << 8) | (x >> 8)) & 0xFFFF


def typecast(x, t):
    """

    :param x: A number or array of numbers to be typecasted.
    :param t: The desired new type. 16 would produce a unsigned 16-bit integer or array of integers.
    :return: The given number typecasted to its new type. The 16-bit hex number 0xFACE typecasted to 8-bit would return
    a 8-bit array containing [0xFA, 0xCE]. Likewise the 8-bit array [0xDE, 0xAD, 0xBE, 0xEF] typecasted to 32-bit would
    return the 32-bit integer 0xDEADBEEF.
    """
    if t == 8:
        x.dtype = np.uint8
    elif t == 16:
        x.dtype = np.uint16
    elif t == 32:
        x.dtype = np.uint32
    return x


def getConf(rcmIp):
    """

    :param rcmIp: The IP of the RCM whose configuration is sought.
    :return: The requested configuration of the RCM.
    """
    port = 21210  # rcm port
    config = np.empty([])  # 13 because we have 13 attributes to config

    # Creating the message
    # creates a 16but unsigned int array from hex
    MSG_TYPE = np.array([int('0002', 16)],dtype=np.uint16)
    # creates a 16but unsigned int array from hex
    MSG_ID = np.array([int('0000', 16)],dtype=np.uint16)
    # swaps 16bit unsigned int bytes and typecasts into 8bit unsigned int
    RCM_GET_CONFIG_REQUEST = typecast(np.array([swapbytes16(MSG_TYPE[0]),
                                                swapbytes16(MSG_ID[0])], dtype=np.uint16), 8)
    # creates a byte array from the uint8 array
    RCM_GET_CONFIG_REQUEST = bytearray(RCM_GET_CONFIG_REQUEST)

    # Setting up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # DGRAM since it' UDP
    s.sendto(RCM_GET_CONFIG_REQUEST, (rcmIp, port))
    timeout = 200  # time in ms
    PACKET_LENGTH = 32  # size in bytes
    s.settimeout(timeout)
    msg, msgAddr = s.recvfrom(PACKET_LENGTH)
    msg = bytearray(msg)  # creates a byte array from the received message

    # Processing message
    msgType = typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
    if msgType != np.ushort(0x0102):
        print 'Message type %04x does not match RCM_GET_CONFIG_CONFIRM.\n', msgType
        config = np.empty(0)
    else:
        configMsgID = typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)  # config.msgID, 0
        configNodeID = typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)  # config.nodeID, 1
        configPII = typecast(np.array([msg[9], msg[8]], dtype=np.uint8), 16)  # config.PII, 2
        configAntMode = np.array([msg[10]], dtype=np.uint8)  # config.antMode, 3
        configCodeChnl = np.array([msg[11]], dtype=np.uint8)  # config.codeChnl, 4
        configAntDlyA = typecast(np.array([msg[15], msg[14], msg[13], msg[12]], dtype=np.uint8), 32)  # config.antDlyA, 5
        configAntDlyB = typecast(np.array([msg[19], msg[18], msg[17], msg[16]], dtype=np.uint8), 32)  # config.antDlyB, 6
        configFlags = typecast(np.array([msg[21], msg[20]], dtype=np.uint8), 16)  # config.flags, 7
        configTxPwr = np.array([msg[22]], dtype=np.uint8)  # config.txPwr, 8
        configUnused = np.array([msg[23]], dtype=np.uint8)  # unused, 9
        configTimeStamp = typecast(np.array([msg[27], msg[26], msg[25], msg[24]], dtype=np.uint8), 32)  # config.timeStamp, 10
        configStat = typecast(np.array([msg[31], msg[30], msg[29], msg[28]], dtype=np.uint8), 32)  # config.stat, 11
        configPersistFlag = np.array([0], dtype=np.uint8)  # 12

        config = [configMsgID, configNodeID, configPII, configAntMode,
                  configCodeChnl, configAntDlyA, configAntDlyB, configFlags,
                  configTxPwr, configUnused, configTimeStamp, configStat, configPersistFlag]
    return s, config
