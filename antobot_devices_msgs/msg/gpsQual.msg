time stamp          # Timestamp
float32 t_offset    # Time offset between GPS and system time
float32 hAcc        # Horizontal accuracy
uint8 gpsQualVal    # Quality value
uint8 numSats       # Number of satellites
float32 horDil      # HDOP
float32 geoSep      # Geoid separation
uint8[] satInfo     # Full satellite information
GnssSystem[] used_systems   # Satellites used in the solution, per constellation (1.2.0 PRN block)
float32 vCOG        # Velocity - Course over ground (heading) [degrees]
float32 vSOG        # Velocity - Speed over ground (km/h)
float32 frequency   # frequency
