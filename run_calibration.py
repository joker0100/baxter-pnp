#!/usr/bin/env python

#from tasks.calibrate import Calibration
from tasks.yumi_calibrate import Calibration


if __name__ == '__main__':
    calibration = Calibration()
    calibration.run()
