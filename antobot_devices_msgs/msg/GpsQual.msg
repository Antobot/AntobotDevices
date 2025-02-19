builtin_interfaces/Time stamp          # Timestamp
float32 t_offset    # Time offset between GPS and system time
float32 h_acc        # Horizontal accuracy
uint8 gps_qual_val    # Quality value
uint8 num_sats       # Number of satellites
float32 hor_dil      # HDOP
float32 geo_sep      # Geoid separation
uint8[] sat_info     # Full satellite information
float32 v_cog        # Velocity - Course over ground (heading) [degrees]
float32 v_sog        # Velocity - Speed over ground (km/h)
