# Snapshot of ALL satellites in view, grouped by constellation.
# Sent periodically (~1 Hz). The satellites actually USED in the position
# solution are reported separately (see the GNSS position message PRNs block).
# Maps to uRCU Data Exchange "GNSS Constellation Message" 1.2.1 (1089).
time stamp                # UTC time of the snapshot
GnssSystem[] systems
