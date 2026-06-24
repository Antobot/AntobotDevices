# A single satellite vehicle in view (one entry of a GSV report).
# Maps to the SV Block of uRCU Data Exchange GNSS Constellation Message 1.2.1.
uint8 svid          # Satellite ID / PRN within its constellation
int16 elevation     # Elevation in degrees (0..90), -1 if unknown
uint16 azimuth      # Azimuth in degrees (0..359), 0 if unknown
uint8 cno           # Carrier-to-noise density (SNR) in dB-Hz, 0 if not tracked
