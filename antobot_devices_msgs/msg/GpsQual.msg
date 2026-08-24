builtin_interfaces/Time stamp          # Timestamp
float32 t_offset    # Time offset between GPS and system time
float32 h_acc        # Horizontal accuracy
uint8 gps_qual_val    # Quality value
uint8 num_sats       # Number of satellites
float32 hor_dil      # HDOP
float32 geo_sep      # Geoid separation
uint8[] sat_info     # Full satellite information
GnssSystem[] used_systems   # Satellites used in the solution, per constellation (1.2.0 PRN block)
float32 corr_age     # Age of differential corrections (s), -1 if not available (1.2.0 Differential block)
int16 station_id     # Differential reference station ID, -1 if not available (1.2.0 Differential block)
float32 v_cog        # Velocity - Course over ground (heading) [degrees]
float32 v_sog        # Velocity - Speed over ground (km/h)
float32 frequency    # frequency