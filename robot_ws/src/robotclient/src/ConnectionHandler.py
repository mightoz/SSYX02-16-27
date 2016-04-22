PKG = 'numpy'
import numpy as np
import socket
import MessageHandler
import LocateRobot


class ConnectionRequest(object):

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.req_ip = None
        self._port = 21210
        self.config_handler = MessageHandler.ConfigHandler()

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

    def meas_range(self, responder_id, do_print):
        """

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
            msg_id = (msg_id + 1) % (0xffff+1)  # contains info about how many times the range have been requested.
            # checks if the RCM is ready to transmit the measured range.
            try:
                status, msg_id_cfrm = MessageHandler.req_range(self.s, self.req_ip,
                                                               self._port, msg_id, responder_id)
            except TypeError:
                print 'RCMSendRangeRequest.req_range returned a NoneType value'
                return
            attempt += 1
            if status[0] == 0:
                # receive information about the measured range and if it was successful.
                try:
                    range_info_status, range_info_fre = MessageHandler.rcm_minimal_range_info(self.s)
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

    def run_loc_rob(self, anchors, nbr_of_success_readings, max_tol, do_print):
        """

        :param anchors: A list oftThe instances of the Anchor class used
        :param nbr_of_success_readings: The number of successful range measurement to each of the nodes. Three nodes and four
        success readings would produce 12 measurements in total.
        :param max_tol: A list of maximum average residual in position as well as maximum average difference between
        last residual in position and current residual that the user is satisfied with.
        :param do_print: 1 to output information about the measuring process, 0 to don't output any information. This
        parameter controls whether or not to plot circles around the nodes with the radius being the measured distances to
        the robots.
        :return: A matrix containing the positions of the robots. The first row contains the x-coordinates of the robots
        and the second row contains the y-coordinates of the robots.
        """

        # instantiate the row vector (N-by-1 matrix) that will hold the measured distance to the nodes.
        # The order that these distances will be in corresponds to the order of the anchors parameter.
        distance = np.zeros((np.size(anchors), 1), dtype=np.float)
        for i in range(0, nbr_of_success_readings):
            for j in range(0, np.size(anchors)):
                if anchors[j].get_ip() is not None and self.req_ip is not None:
                    dist = self.meas_range(anchors[j].get_ip(), do_print)  # get the distance data.
                    distance[j, 0] += dist / nbr_of_success_readings  # add the average distance.
                else:
                    print 'Check the ip of anchor %s and UWB radio on the robot' % j
                    return
        pos = LocateRobot.locate_robot(anchors, distance, max_tol)
        return pos

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
