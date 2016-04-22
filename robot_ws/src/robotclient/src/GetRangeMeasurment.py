#!/usr/bin/env python
PKG = 'numpy'
import RCMSendRangeRequest
import numpy as np


def meas_range(s, requester_ip, responder_id, do_print):
    """

    :param s: The socket that is used to communicate between computer and RCM through a wired connection
    :param requester_ip: The ip of the RCM that is connected via an ethernet cable.
    :param responder_id: The id of the node which distance to the RCM is sought.
    :param do_print: Not used at this moment. In the future this variable may be used to decide whether or not some
    technical information about the measuring process should be printed
    :return: The measured range between the node and the robot.
    """
    msg_id = 0
    attempt = 0
    success = 0
    calc_range = 0
    while success < 1:
        msg_id = (msg_id + 1) % (0xffff+1)  # contains information about how many times the range have been requested.
        # checks if the RCM is ready to transmit the measured range.
        try:
            status, msg_id_confirm = RCMSendRangeRequest.req_range(s, requester_ip, msg_id, responder_id)
        except TypeError:
            print 'RCMSendRangeRequest.req_range returned a NoneType value'
            return
        attempt += 1
        if status[0] == 0:
            # receive information about the measured range and if it was successful.
            try:
                range_info_status, range_info_fre = RCMSendRangeRequest.rcm_minimal_range_info(s)
            except TypeError:
                print 'RCMSendRangeRequest.rcm_minimal_range_info returned a NoneType value'
                return
            if range_info_status[0] == 0:  # successful measurement
                success = 1
                calc_range = range_info_fre[0]/1000.0
            elif range_info_status[0] == 1:
                print 'range timeout\n'
            elif range_info_status[0] == 2:
                print 'LED failure\n'
            elif range_info_status[0] == 9:
                print 'UDP failure on InfoReceive\n'
            else:
                print 'UDP failure on InfoReceive\n'

        if attempt > 10:
            print 'Error in measuring range'
            return

    return calc_range

