#!/usr/bin/env python3

"""
This python script should be run in the base station connected via USB to analyse the performance of base station
It will take a minute or two after powering the base station to get into FIX mode

Dependeny required to run this script: python>=3.8
install the dependecy: python3 -m pip install --upgrade pyubx2
contact : Aswathi Muralidharan
"""
import sys
from serial import Serial
from pyubx2 import UBXReader
stream = Serial(port='/dev/ttyUSB0', baudrate=38400)

def main(argv):
    try:
        while True:
            stream.flush()
            ubr = UBXReader(stream)
            (raw_data,parsed_data) = ubr.read()
            print("Parsed data: ",parsed_data)
            if type(parsed_data) is not str:
                if parsed_data.identity == 'NAV-PVT':
                    #print("Parsed data: ",type(parsed_data))
                    fix_type = parsed_data.fixType
                    #print("Parsed data: ",parsed_data)
                    #print("Fix type is: ",fix_type)
                    if fix_type == 5:
                        print("Base station in RTK-TIME FIX")
                        print("Latitude is : ", parsed_data.lat)
                        print("Longitude is: ",parsed_data.lon)
                        print("Number of satellites available: ", parsed_data.numSV)
                        print("Horizontal Accuracy in mm: ",parsed_data.hAcc)
                    else:
                        print("Base station in not in FIX mode. Try again after one minute")
                        print("Fix type is: ",fix_type)
                else:
                    print("Wait for some time")
                    pass

    except KeyboardInterrupt:
        print("Terminated by keyboard interrupt")

if __name__ == '__main__':
    main(sys.argv)
