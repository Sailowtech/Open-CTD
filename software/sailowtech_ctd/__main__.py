#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This program aims to facilitate communication with multiple sensors inside a CTD.
For now, it supports I2C protocol but there should be more in the future.

Credits for I2C protocol: "https://github.com/Atlas-Scientific/Raspberry-Pi-sample-code/blob/master/AtlasI2C.py"]
Contact                 : "arthur.jacobs@sailowtech.ch"
"""
from datetime import datetime
import time
from pprint import pprint

from types_.ctd import CTD

if __name__ == '__main__':
    ctd = CTD()
    # ctd.setup_sensors()

    pprint(ctd.sensors)

    # input("Appuyez sur Entrée pour procéder à la calibration des capteurs...\n")
    # ctd.calibrate_sensors()

    ctd.pressure_threshold = int(input("Threshold de coupure des mesures (mba)(généralement 200 à 500 mbars) : "))

    i = 0
    while i < 30 and ctd.activated:
        ctd.measure_all()
        time.sleep(1)
        i += 1

    str_now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    ctd.export_csv(f'{str_now}_data.csv')
    print("Exported csv, exiting program")
