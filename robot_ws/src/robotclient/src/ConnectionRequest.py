PKG = 'numpy'
import numpy as np
import ConfigHandler
import socket


class ConnectionRequest(object):

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.req_ip = None
        self._port = 21210
        self.config_handler = ConfigHandler.ConfigHandler()

    def set__rcm_ip(self, val):
        self.req_ip = '192.168.1.'+str(val)

    def get_socket(self):
        return self.s

    def get_rcm_ip(self):
        return self.req_ip

    def get_port(self):
        return self._port

    def get_config_handler(self):
        return self.config_handler

    def connect_req(self, do_print):
        """

        :param req_id: The ID of the RCM:s (wired) ip. If the RCM:s ip 192.168.1.XXX, then XXX is the id
        :param do_print: The config will be printed if do_print is 1. It will not be printed if it is 0.
        :return: The socket that has been created and the ip of the RCM that has been constructed from its id.
        """
        if self.req_ip is not None:
            status = self.config_handler.get_configuration(self.s, self.req_ip, self._port)
            if status == -1:
                return -1
        else:
            print 'UWB radio has no ip set'
            return -1

        """
        Uncomment this to change configuration parameters.
        Leave commented to accept the defaults from the radio
        """
        # self.config_handler.config.pii = np.array([7], dtype=np.uint16)  # This must match the responder
        # self.config_handler.config.code_chnl = np.array([6], dtype=np.uint8)  # This must match the responder
        # self.config_handler.config.tx_pwr = np.array([10], dtype=np.uint8)
        self.config_handler.config.flags = np.array([1], dtype=np.uint16)  # make sure scan data is set (not default)
        self.config_handler.config.persist_flag = np.array([0], dtype=np.uint8)  # Don't change this in flash
        if self.config_handler.config.node_id is not None:
            self.config_handler.set_conf(self.s, self.req_ip, self._port)
        else:
            print 'Unable to parse config, check your connection with the UWB'
            return -1
        if do_print:
            print self.config_handler.config
        return

    def dc_req(self, do_print):
        """

        :param do_print: A message saying that the socket is closing if this parameter is 1. The message will not be
        printed if it is 0.
        :return:
        """
        if do_print:
            print 'closing socket'
        if self.s is not None:
            self.s.close()
        return
