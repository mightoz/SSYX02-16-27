#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import MiscFunctions as Mf


def req_range(s, requester_ip, msg_id, responder_id):
    """

    :param s: The socket that is used to communicate between the computer and the RCM connected via ethernet cable.
    :param requester_ip: The IP of the RCM whose configuration is sought to set.
    :param msg_id: ID of the message to send to the RCM. The id will decide what type of measurement
    the RCM will perform.
    :param responder_id: The id of the node to which measurements will be done.
    :return: The status of the request and the confirmed message id from the RCM. This message should be the same
    as the one you sent it to make sure you received what you want.
    """
    port = 21210

    status = np.array([0xFFFFFFFF], dtype=np.uint32)  # return variable 1
    msg_id_confirm = np.empty(0, dtype=np.uint32)  # return variable 2

    msg_type = np.array([int('0003', 16)], dtype=np.uint16)  # rcm_send_range_request message type.
    msg_id = np.array([msg_id], dtype=np.uint16)
    resp_id = np.array([responder_id], dtype=np.uint32)
    ant_mode = np.array([0], dtype=np.uint8)
    reserved = np.array([0], dtype=np.uint8)
    data_size = np.array([0], dtype=np.uint16)
    data = np.empty(0, dtype=np.uint8)

    rcm_send_range_request = np.concatenate([Mf.typecast(Mf.swap_bytes_16(msg_type), 8),
                                             Mf.typecast(Mf.swap_bytes_16(msg_id), 8),
                                             Mf.typecast(Mf.swap_bytes_32(resp_id), 8),
                                             ant_mode, reserved,
                                             Mf.typecast(Mf.swap_bytes_16(data_size), 8),
                                             data])
    rcm_send_range_request.dtype = np.uint8
    rcm_send_range_request = bytearray(rcm_send_range_request)

    # send data
    s.sendto(rcm_send_range_request, (requester_ip, port))
    timeout = 300
    _packet_length = 8
    s.settimeout(timeout)
    msg, msg_addr = s.recvfrom(_packet_length)
    msg = bytearray(msg)  # Unpack string to byte array

    # processing message
    msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
    if msg_type != np.array([int('0103', 16)], dtype=np.uint16):
        print 'Message type %s does not match RCM_SEND_RANGE_CONFIRM. ' % msg_type
        status = np.array([1], dtype=np.uint32)
        # msg_id_confirm remains empty uint32
    else:
        msg_id_confirm = Mf.typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
        status = Mf.typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
    return status, msg_id_confirm


def rcm_minimal_range_info(s):
    """

    :param s: The socket that is used to communicate between the computer and the RCM connected via ethernet cable.
    :return: The status of the measurement and the measured distance between the RCM and the node. This distance is
    given in millimeters.
    """
    range_info_status = np.zeros(1, dtype=np.uint8)
    range_info_fre = np.zeros(1, dtype=np.double)

    timeout = 500  # ms
    packet_length = 2048  # Largest expected UDP packet (bytes)

    rng_info_rcvd = False

    while not rng_info_rcvd:
        s.settimeout(timeout)
        msg, msg_addr = s.recvfrom(packet_length)
        msg = bytearray(msg)  # Unpack string to byte array
        msg_type = np.array([msg[1], msg[0]], dtype=np.uint8)
        msg_type.dtype = np.uint16
        if msg_type == np.array([int('0201', 16)], dtype=np.uint16):
            range_info_status[0] = msg[8]  # rangeInfo.status
            tmp0 = np.array([msg[23], msg[22], msg[21], msg[20]], dtype=np.uint8)
            tmp0.dtype = np.uint32
            range_info_fre[0] = np.double(tmp0)  # rangeInfo.fre
            rng_info_rcvd = True

    return range_info_status, range_info_fre
