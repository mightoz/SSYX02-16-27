import numpy as np


def swap_bytes_16(x):
    """

    :param x: A 16-bit unsigned integer or array of integers.
    :return: The reverse order of the bytes of the given number or array.
    The given number 0xABCD would return 0xCDAB.
    """
    if x is not None:
        return ((x << 8) | (x >> 8)) & 0xFFFF
    else:
        return x


def swap_bytes_32(x):
    """

    :param x: A 32-bit unsigned integer or array of integers.
    :return: The reverse order of the bytes of the given number or array.
    The given number 0x89ABCDEF would return 0xEFCDAB89.
    """
    if x is not None:
        return (((x << 24) & 0xFF000000) |
                ((x << 8) & 0x00FF0000) |
                ((x >> 8) & 0x0000FF00) |
                ((x >> 24) & 0x000000FF))
    else:
        return x


def typecast(x, t):
    """

    :param x: A number or array of numbers to be typecasted.
    :param t: The desired new type. 16 would produce a unsigned 16-bit
    integer or array of integers.
    :return: The given number typecasted to its new type. The 16-bit hex
    number 0xFACE typecasted to 8-bit would return a 8-bit array containing
    [0xFA, 0xCE]. Likewise the 8-bit array [0xDE, 0xAD, 0xBE, 0xEF]
    typecasted to 32-bit would return the 32-bit integer 0xDEADBEEF.
    """
    if x is not None:
        if t == 8:
            x.dtype = np.uint8
        elif t == 16:
            x.dtype = np.uint16
        elif t == 32:
            x.dtype = np.uint32
    return x
