#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import socket
import Config
import MiscFunctions as Mf


class ConfigHandler(object):

    def __init__(self):
        self.config = Config.Config()

    def get_config_msg(self):
        return self.config

    def get_configuration(self, s, ip, port):
        """

        :param s: connection socket
        :param ip: rcm ethernet ip
        :param port: connection port
        :return:
        """

        # Creating the message
        msg_type = np.array([int('0002', 16)], dtype=np.uint16)
        msg_id = np.array([int('0000', 16)], dtype=np.uint16)
        rcm_get_config_request = Mf.typecast(np.array([Mf.swap_bytes_16(msg_type[0]),
                                                    Mf.swap_bytes_16(msg_id[0])], dtype=np.uint16), 8)
        rcm_get_config_request = bytearray(rcm_get_config_request)

        try:
            s.sendto(rcm_get_config_request, (ip, port))
            timeout = 0.2  # time in s
            _packet_length = 32  # size in bytes
            s.settimeout(timeout)
            msg, msg_addr = s.recvfrom(_packet_length)
        except socket.timeout:
            print 'connection timed out after %s seconds' % timeout
            return -1
        msg = bytearray(msg)

        # Processing message
        msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
        if msg_type != np.array([0x0102], dtype=np.uint16):
            print 'Message type %s does not match RCM_GET_CONFIG_CONFIRM.' % msg_type
            return -1
        else:
            self.config.msg_id = Mf.typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
            self.config.node_id = Mf.typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
            self.config.pii = Mf.typecast(np.array([msg[9], msg[8]], dtype=np.uint8), 16)
            self.config.ant_mode = np.array([msg[10]], dtype=np.uint8)
            self.config.code_chnl = np.array([msg[11]], dtype=np.uint8)
            self.config.ant_dly_a = Mf.typecast(np.array([msg[15], msg[14], msg[13], msg[12]], dtype=np.uint8), 32)
            self.config.ant_dly_b = Mf.typecast(np.array([msg[19], msg[18], msg[17], msg[16]], dtype=np.uint8), 32)
            self.config.flags = Mf.typecast(np.array([msg[21], msg[20]], dtype=np.uint8), 16)
            self.config.tx_pwr = np.array([msg[22]], dtype=np.uint8)
            self.config.unused = np.array([msg[23]], dtype=np.uint8)
            self.config.time_stamp = Mf.typecast(np.array([msg[27], msg[26], msg[25], msg[24]], dtype=np.uint8), 32)
            self.config.stat = Mf.typecast(np.array([msg[31], msg[30], msg[29], msg[28]], dtype=np.uint8), 32)
            self.config.persist_flag = np.array([0], dtype=np.uint8)
        return

    def set_conf(self, s, ip, port):
        """

        :param s: The socket that is used to communicate between the computer and the RCM connected via ethernet cable.
        :param ip: The IP of the RCM whose configuration is sought to set.
        :param port: connection port
        :return: the status of the response from the RCM.
        """
        msg_type = Mf.swap_bytes_16(np.array([int('0001', 16)], dtype=np.uint16))  # rcm_set_config_request message type.
        msg_id = Mf.swap_bytes_16(np.array([int('0003', 16)], dtype=np.uint16))
        node_id = Mf.swap_bytes_32(self.config.node_id)
        pii = Mf.swap_bytes_16(self.config.pii)
        ant_mode = self.config.ant_mode
        code = self.config.code_chnl
        ant_delay_a = Mf.swap_bytes_32(self.config.ant_dly_a)
        ant_delay_b = Mf.swap_bytes_32(self.config.ant_dly_b)
        flags = Mf.swap_bytes_16(self.config.flags)
        tx_gain = self.config.tx_pwr
        persist = self.config.persist_flag

        rcm_set_config_request = np.concatenate([Mf.typecast(np.array([msg_type[0], msg_id[0]], dtype=np.uint16), 8),
                                                 Mf.typecast(np.array([node_id[0]], dtype=np.uint32), 8),
                                                 Mf.typecast(np.array([pii[0]], dtype=np.uint16), 8),
                                                 ant_mode, code,
                                                 Mf.typecast(np.array([ant_delay_a[0], ant_delay_b[0]], dtype=np.uint32), 8),
                                                 Mf.typecast(np.array([flags[0]], dtype=np.uint16), 8),
                                                 tx_gain, persist])
        rcm_set_config_request.dtype = np.uint8
        rcm_set_config_request = bytearray(rcm_set_config_request)

        # send data
        try:
            s.sendto(rcm_set_config_request, (ip, port))
            timeout = 0.4  # time in s
            _packet_length = 8  # size in byte
            s.settimeout(timeout)
            msg, msg_addr = s.recvfrom(_packet_length)
        except socket.timeout:
            print 'connection timed out after %s seconds' % timeout
            return
        msg = bytearray(msg)  # Unpack string to byte array

        # processing response
        msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
        if msg_type != np.array([int('0101', 16)], dtype=np.uint16):
            print 'Message type %s does not match RCM_SET_CONFIG_CONFIRM. ' % msg_type
        else:
            # msg_id in confirm should be equal to msg_id in request
            msg_id = Mf.typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
            # status = typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
        return msg_id
