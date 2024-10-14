### antobot_devices_gps

The antobot_devices_gps repository outlook:
* config
  *ppp_configuration yaml file
  *PPP certificate pem and crt file, to be downloaded from thing web
* src
  * gps_ppp.py: Python script for F9P in uRCU via SPI using the sparkfun library; this script is used when uRCU is using PPP-IP
  * base_station_test: analyse the performance of base station via USB
  * f9p_config.py: Python script to configure the F9P in uRCU via SPI. just need to run once for new uURCU.
  * gps_strength_test: script to test the number of satellite and strength of GPS signal
  * set_gpio_high: script to set xavier's GPIO01 ping as high to enable PPP. for test purpose 
  * antobot_devices_gps:
    * third party dependencies to run the gps_ppp.py (LICENCE provided)
* setup.py: installation file for antobot_gps_urcu



## How to use this repository:

* Always configure the F9P chip before it.

* run 'f9p_config.py' to configure the F9P. This will give 8Hz GPS frequency. To make changes to the configuration, make sure that the new configurations are written to both flash and ram memory of F9P. we use only GGA message types [previous name gps_config.py]

* 'gps_base_station.py' is used in uRCU for gps-spi communication   [previous name am_base_station.py]

* 'gps_ppp.py' to use the PPP-IP 8Hz F9P. this works only in HPG1.32 or 04B version of F9P. This script is for uRCU version 1.2C above with nVidia jetson package 35.1.0   [previous name antobot_nRTK_rover.py]

* 'gps_base_station.py' is used to launch gps with base station [previous name antobot_gps.py] 

********* below script has been removed since version 1.6.1 ***********
* 'antobot_gps1.py' was developed to toggle between the base station script and ppp-ip script upon the button press of 'G'. But this script was never tested or used.

* use 'antobot_gps_nRTK.py' to get the PPP-IP 8Hz F9P. this works only in HPG1.32 or 04B version of F9P. This script is for USB communication







