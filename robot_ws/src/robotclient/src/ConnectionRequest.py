PKG = 'numpy'
import numpy as np
import rcmGetConfig
import rcmSetConfig


def connectReq(reqId, doPrint):
    """

    :param reqId: The ID of the RCM:s (wired) ip. If the RCM:s ip 192.168.1.XXX, then XXX is the id
    :param doPrint: The config will be printed if doPrint is 1. It will not be printed if it is 0.
    :return: The socket that has been created and the ip of the RCM that has been constructed from its id.
    """
    reqIp = '192.168.1.'+str(reqId)
    [s, config] = rcmGetConfig.getConf(reqIp)

    """ Uncomment this to change configuration parameters.
        Leave commented to accept the defaults from the radio """
    # config[2] = 7  # config.pii  # This must match the responder
    # config[4] = 6  # config.codeChnl  # This must match the responder
    # config[8] = 10;  # config.txPwr
    config[7] = np.array([1], dtype=np.uint16)  # configFlags, make sure scan data is set (not default)
    config[12] = np.array([0], dtype=np.uint8)  # configPersistFlag, Don't change this in flash

    status = rcmSetConfig.setConf(s, reqIp, config)
    if doPrint:
        print config
    return [s, reqIp]


def dcReq(s, doPrint):
    """

    :param s: The socket that is t obe closed
    :param doPrint: A message saying that the socket is closing if this parameter is 1. The message will not be
    printed if it is 0.
    :return:
    """
    if doPrint:
        print 'closing socket'
    s.close
    return
