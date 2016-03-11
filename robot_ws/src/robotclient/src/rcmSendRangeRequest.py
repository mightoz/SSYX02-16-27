#!/usr/bin/env python
PKG = 'numpy'
import numpy as np

def swapbytes16(x):
    """

    :param x: A 16-bit unsigned integer or array of integers.
    :return: The reverse order of the bytes of the given number or array.
    The given number 0xABCD would return 0xCDAB.
    """
    return ((x << 8) | (x >> 8)) & 0xFFFF


def swapbytes32(x):
    """

    :param x: A 32-bit unsigned integer or array of integers.
    :return: The reverse order of the bytes of the given number or array.
    The given number 0x89ABCDEF would return 0xEFCDAB89.
    """
    return (((x << 24) & 0xFF000000) |
            ((x << 8) & 0x00FF0000) |
            ((x >> 8) & 0x0000FF00) |
            ((x >> 24) & 0x000000FF))


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


def reqRange(s, rcmIp, msgId, rangeRespId):
    """

    :param s: The socket that is used to communicate between the computer and the RCM connected via ethernet cable.
    :param rcmIp: The IP of the RCM whose configuration is sought to set.
    :param msgId: ID of the message to send to the RCM. The id will decide what type of measurement
    the RCM will perform.
    :param rangeRespId: The id of the node to which measurements will be done.
    :return: The status of the request and the confirmed message id from the RCM. This message should be the same
    as the one you sent it to make sure you recieved what you want.
    """
    port = 21210

    status = np.array([0xFFFFFFFF], dtype=np.uint32)  # return variable 1
    msgIdCfrm = np.empty(0, dtype=np.uint32)  # return variable 2

    MSG_TYPE = np.array([int('0003', 16)], dtype=np.uint16)  # RCM_SEND_RANGE_REQUEST message type.
    MSG_ID = np.array([msgId], dtype=np.uint16)
    RESP_ID = np.array([rangeRespId], dtype=np.uint32)
    ANT_MODE = np.array([0], dtype=np.uint8)
    RESERVED = np.array([0], dtype=np.uint8)
    DATA_SIZE = np.array([0], dtype=np.uint16)
    DATA = np.empty(0, dtype=np.uint8)

    RCM_SEND_RANGE_REQUEST = np.concatenate([typecast(swapbytes16(MSG_TYPE), 8),
                                       typecast(swapbytes16(MSG_ID), 8),
                                       typecast(swapbytes32(RESP_ID), 8),
                                       ANT_MODE, RESERVED,
                                       typecast(swapbytes16(DATA_SIZE), 8),
                                       DATA])
    RCM_SEND_RANGE_REQUEST.dtype = np.uint8
    RCM_SEND_RANGE_REQUEST = bytearray(RCM_SEND_RANGE_REQUEST)

    # send data
    s.sendto(RCM_SEND_RANGE_REQUEST, (rcmIp, port))
    timeout = 300
    PACKET_LENGTH = 8
    s.settimeout(timeout)
    msg, msgAddr = s.recvfrom(PACKET_LENGTH)
    msg = bytearray(msg)  # Unpack string to byte array

    # processing message
    msgType = typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
    if msgType != np.array([int('0103', 16)], dtype=np.uint16):
        print 'Message type %04x does not match RCM_SEND_RANGE_CONFIRM. ', msgType
        status = np.array([1], dtype=np.uint32)
        # msgIdCfrm remains empty uint32
    else:
        msgIdCfrm = typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
        status = typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
    return status, msgIdCfrm


def rcmMinimalRangeinfo(s):
    """

    :param s: The socket that is used to communicate between the computer and the RCM connected via ethernet cable.
    :return: The status of the measurement and the measured distance between the RCM and the node. This distance is
    given in millimeters.
    """
    rangeInfoStatus = np.zeros(1, dtype=np.uint8)
    rangeInfoFre = np.zeros(1, dtype=np.double)

    timeout = 500  # ms
    PACKET_LENGTH = 2048  # Largest expected UDP packet (bytes)

    rngInfoRcvd = False

    while not rngInfoRcvd:
        s.settimeout(timeout)
        msg, msgAddr = s.recvfrom(PACKET_LENGTH)
        msg = bytearray(msg)  # Unpack string to byte array
        msgType = np.array([msg[1], msg[0]], dtype=np.uint8)
        msgType.dtype = np.uint16
        if msgType == np.array([int('0201', 16)], dtype=np.uint16):
            rangeInfoStatus[0] = msg[8]  # rangeInfo.status
            tmp0 = np.array([msg[23], msg[22], msg[21], msg[20]], dtype=np.uint8)
            tmp0.dtype = np.uint32
            rangeInfoFre[0] = np.double(tmp0)  # rangeInfo.fre
            rngInfoRcvd = True

    return rangeInfoStatus, rangeInfoFre
