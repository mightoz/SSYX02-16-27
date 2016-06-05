PKG = 'numpy'
import numpy as np
import UWBHandler
import LocateRobot


class MessageHandler(object):

    def __init__(self):
        self.uwb_handler = UWBHandler.UWBHandler()

    def set_ip(self, val):
        self.uwb_handler.set__rcm_ip(val)

    def get_config_handler(self):
        return self.uwb_handler

    def connect_req(self, do_print):
        """

        :param do_print: The config will be printed if do_print is 1.
        It will not be printed if it is 0.
        :return: The socket that has been created and the ip of the RCM
        that has been constructed from its id.
        """
        status = self.uwb_handler.get_configuration()
        if status == -1:
            return -1
        """
        Uncomment this to change configuration parameters. Leave commented
        to accept the defaults from the radio
        # This must match the responder
        self.uwb_handler.config.pii = np.array([7], dtype=np.uint16)
        # This must match the responder
        self.uwb_handler.config.code_chnl = np.array([6], dtype=np.uint8)
        self.uwb_handler.config.tx_pwr = np.array([10], dtype=np.uint8)
        """
        # make sure scan data is set (not default)
        self.uwb_handler.config.flags = np.array([1], dtype=np.uint16)
        # Don't change this in flash
        self.uwb_handler.config.persist_flag = np.array([0], dtype=np.uint8)
        status = self.uwb_handler.set_conf()
        if status == -1:
            return -1
        if do_print:
            print self.uwb_handler.config
        return

    def meas_range(self, responder_id, do_print):
        """

        :param responder_id: The id of the node which distance to the RCM
        is sought.
        :param do_print: Not used at this moment. In the future this
        variable may be used to decide whether or not some technical
        information about the measuring process should be printed
        :return: The measured range between the node and the robot.
        """
        msg_id = 0
        attempt = 0
        success = 0
        calc_range = 0
        while success < 1:
            # contains info about how many times the range have been requested.
            msg_id = (msg_id + 1) % (0xffff+1)
            
            # checks if the RCM is ready to transmit the measured range.
            try:
                status, msg_id_cfrm = self.uwb_handler.req_range(msg_id,
                                                                 responder_id)
            except TypeError:
                print 'RCMSendRangeRequest.req_range returned a NoneType value'
  
            attempt += 1
            if status[0] == 0:
                """
                receive information about the measured range and if it
                was successful.
                """
                try:
                    range_info_status, range_info_fre = self.uwb_handler.rcm_minimal_range_info()
                except TypeError:
                    print 'RCMSendRangeRequest.rcm_minimal_range_info returned a NoneType value'

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
                return -1

        return calc_range

    def run_loc_rob(self, anchors, nbr_of_success_readings, max_tol, do_print):
        """

        :param anchors: A list oftThe instances of the Anchor class used
        :param nbr_of_success_readings: The number of successful range
        measurement to each of the nodes. Three nodes and four success readings
        would produce 12 measurements in total.
        :param max_tol: A list of maximum average residual in position as well
        as maximum average difference between last residual in position and
        current residual that the user is satisfied with.
        :param do_print: 1 to output information about the measuring process,
        0 to don't output any information. This
        parameter controls whether or not to plot circles around the nodes
        with the radius being the measured
        distances to the robots.
        :return: A matrix containing the positions of the robots. The first
        row contains the x-coordinates of the robots
        and the second row contains the y-coordinates of the robots.
        """
        """
        Instantiate the row vector (N-by-1 matrix) that will hold the measured
        distance to the nodes. The order that these distances will be in
        corresponds to the order of the anchors parameter.
        """
        failed = False
        distance = np.zeros((np.size(anchors), 1), dtype=np.float)
        for i in range(0, nbr_of_success_readings):
            for j in range(0, np.size(anchors)):
                if anchors[j].get_ip() is not None:
                    # get the distance data.
                    dist = self.meas_range(anchors[j].get_ip(), do_print)
                    if dist != -1:
                        # add the average distance.
                        distance[j, 0] += dist / nbr_of_success_readings
                    else:
                        failed = True
                        break
                else:
                    print 'Check the ip of anchor %s' % j
                    failed = True
        finalpos = None
        if failed is False:
            pos = LocateRobot.locate_robot(anchors, distance,
                                           max_tol, do_print)
            if pos[0] == 1337:
                finalpos = None
            else:
                finalpos = pos
        return finalpos

    def dc_req(self, do_print):
        """

        :param do_print: A message saying that the socket is
        closing if this parameter is True.
        The message will not be printed if it is False.
        :return:
        """
        self.uwb_handler.disconnect(do_print)
        return
