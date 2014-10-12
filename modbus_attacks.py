#!/usr/bin/env python
# vim: tw=80

from ..protolibs.modbus_tk import utils as modbus_utils

"""

Modbus Attacks
This module holds attacks against ModbusTCP and ModbusASCII slaves and masters.
"""

class Attack(object):
    def runAttack(*args, **kwds):
        raise NotImplementedError

class ModbusAttack(Attack):
    pass

class ModbusRTUAttack(Attack):
    def createPort(filename):
        """Creates serial port objects with appropriate characteristics"""
        pass
    def bytewiseRead(self, ):


