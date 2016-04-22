PKG = 'numpy'
import numpy as np
import rcmGetConfig
import rcmSetConfig
import socket


class ConnectionRequest(object):

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.req_ip = None
        self.config = None

    def set_req_ip(self, val):
        if val[0:10] == '192.168.1.':
            self.req_ip = val
        else:
            print 'Invalid IP'

    def get_socket(self):
        return self.s

    def get_req_ip(self):
        return self.req_ip

    def connect_req(self, do_print):
        """

        :param req_id: The ID of the RCM:s (wired) ip. If the RCM:s ip 192.168.1.XXX, then XXX is the id
        :param do_print: The config will be printed if do_print is 1. It will not be printed if it is 0.
        :return: The socket that has been created and the ip of the RCM that has been constructed from its id.
        """

        while self.config is None:
            self.s, self.config = rcmGetConfig.getConf(self.req_ip)

        """
        Uncomment this to change configuration parameters.
        Leave commented to accept the defaults from the radio
        """
        # config[2] = 7  # config.pii  # This must match the responder
        # config[4] = 6  # config.codeChnl  # This must match the responder
        # config[8] = 10;  # config.txPwr
        self.config[7] = np.array([1], dtype=np.uint16)  # configFlags, make sure scan data is set (not default)
        self.config[12] = np.array([0], dtype=np.uint8)  # configPersistFlag, Don't change this in flash

        rcmSetConfig.setConf(self.s, self.req_ip, self.config)
        if do_print:
            print self.config
        return self.s, self.req_ip

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
