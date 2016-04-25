#!/usr/bin/env python
PKG = 'numpy'
import numpy as np
import socket
import Config
import MiscFunctions as Mf


class UWBHandler(object):

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.req_ip = None
        self._port = 21210
        self.config = Config.Config()

    def set__rcm_ip(self, val):
        self.req_ip = '192.168.1.'+str(val)

    def get_socket(self):
        return self.s

    def get_rcm_ip(self):
        return self.req_ip

    def get_port(self):
        return self._port

    def get_config_msg(self):
        return self.config

    def send_rcv(self, msg, packet_length, timeout):
        """

        :param msg: msg to send
        :param packet_length: packet size in bytes
        :param timeout: timeout time in seconds
        :return: received message
        """
        try:
            self.s.sendto(msg, (self.req_ip, self._port))
            self.s.settimeout(timeout)
            msg, msg_addr = self.s.recvfrom(packet_length)
        except socket.timeout:
            print 'connection timed out after %s seconds' % timeout
        if msg is not None:
            return bytearray(msg)
        else:
            return

    def get_configuration(self):
        """

        :return:
        """
        # Creating the message
        msg_type = np.array([int('0002', 16)], dtype=np.uint16)
        msg_id = np.array([int('0000', 16)], dtype=np.uint16)
        rcm_get_config_request = Mf.typecast(np.array([Mf.swap_bytes_16(msg_type[0]),
                                                       Mf.swap_bytes_16(msg_id[0])], dtype=np.uint16), 8)
        rcm_get_config_request = bytearray(rcm_get_config_request)
        # send data
        if self.req_ip is not None:
            msg = self.send_rcv(rcm_get_config_request, 32, 0.2)
        else:
            print 'UWB radio has no ip set'
            return -1
        # Processing message
        if msg is not None:
            msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
            if msg_type != np.array([0x0102], dtype=np.uint16):
                print 'Message type %s does not match RCM_GET_CONFIG_CONFIRM.' % msg_type
                return
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

    def set_conf(self):
        """

        :return: the status of the response from the RCM.
        """
        msg_type = Mf.swap_bytes_16(np.array([int('0001', 16)], dtype=np.uint16))  # rcm_set_config_request msg type.
        msg_id = Mf.swap_bytes_16(np.array([int('0003', 16)], dtype=np.uint16))
        if self.config.node_id is not None:
            node_id = Mf.swap_bytes_32(self.config.node_id)
        else:
            print 'UWB radio has no ip set'
            return -1
        pii = Mf.swap_bytes_16(self.config.pii)
        ant_mode = self.config.ant_mode
        code = self.config.code_chnl
        ant_delay_a = Mf.swap_bytes_32(self.config.ant_dly_a)
        ant_delay_b = Mf.swap_bytes_32(self.config.ant_dly_b)
        flags = Mf.swap_bytes_16(self.config.flags)
        tx_gain = self.config.tx_pwr
        persist = self.config.persist_flag

        rcm_set_config_request = \
            np.concatenate([Mf.typecast(np.array([msg_type[0], msg_id[0]], dtype=np.uint16), 8),
                            Mf.typecast(np.array([node_id[0]], dtype=np.uint32), 8),
                            Mf.typecast(np.array([pii[0]], dtype=np.uint16), 8),
                            ant_mode, code,
                            Mf.typecast(np.array([ant_delay_a[0], ant_delay_b[0]], dtype=np.uint32), 8),
                            Mf.typecast(np.array([flags[0]], dtype=np.uint16), 8),
                            tx_gain, persist])
        rcm_set_config_request.dtype = np.uint8
        rcm_set_config_request = bytearray(rcm_set_config_request)
        # send data
        if self.req_ip is not None:
            msg = self.send_rcv(rcm_set_config_request, 8, 0.4)
        else:
            print 'Unable to parse config, check your connection with the UWB'
            return -1
        # processing response
        if msg is not None:
            msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
            if msg_type != np.array([int('0101', 16)], dtype=np.uint16):
                print 'Message type %s does not match RCM_SET_CONFIG_CONFIRM. ' % msg_type
            else:
                # msg_id in confirm should be equal to msg_id in request
                msg_id = Mf.typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
                # status = Mf.typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
        return msg_id

    def req_range(self, msg_id, responder_id):
        """

        :param msg_id: ID of the message to send to the RCM. The id will decide what type of measurement
        the RCM will perform.
        :param responder_id: The id of the node to which measurements will be done.
        :return: The status of the request and the confirmed message id from the RCM. This message should be the same
        as the one you sent it to make sure you received what you want.
        """

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
        if self.req_ip is not None:
            msg = self.send_rcv(rcm_send_range_request, 8, 0.3)
        else:
            print 'Unable to parse config, check your connection with the UWB'
            return -1
        # processing message
        if msg is not None:
            msg_type = Mf.typecast(np.array([msg[1], msg[0]], dtype=np.uint8), 16)
            if msg_type != np.array([int('0103', 16)], dtype=np.uint16):
                print 'Message type %s does not match RCM_SEND_RANGE_CONFIRM. ' % msg_type
                status = np.array([1], dtype=np.uint32)
                # msg_id_confirm remains empty uint32
            else:
                msg_id_confirm = Mf.typecast(np.array([msg[3], msg[2]], dtype=np.uint8), 16)
                status = Mf.typecast(np.array([msg[7], msg[6], msg[5], msg[4]], dtype=np.uint8), 32)
        return status, msg_id_confirm

    def rcm_minimal_range_info(self):
        """

        :return: The status of the measurement and the measured distance between the RCM and the node. This distance is
        given in millimeters.
        """
        range_info_status = np.zeros(1, dtype=np.uint8)
        range_info_fre = np.zeros(1, dtype=np.double)

        timeout = 0.5  # s
        packet_length = 2048  # Largest expected UDP packet (bytes)

        rng_info_rcvd = False
        msg = bytearray()
        while not rng_info_rcvd:  # this MUST be here or error msg_id 513 will occur
            try:
                self.s.settimeout(timeout)
                msg, msg_addr = self.s.recvfrom(packet_length)
            except socket.timeout:
                print 'connection timed out after %s seconds' % timeout
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

    def disconnect(self, do_print):
        """

        :param do_print: A message saying that the socket is closing if this parameter is True.
        The message will not be printed if it is False.
        :return:
        """
        if do_print:
            print 'closing socket'
        if self.s is not None:
            self.s.close()
        return
