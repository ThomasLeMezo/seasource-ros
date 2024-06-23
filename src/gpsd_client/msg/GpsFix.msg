# A more complete GPS fix to supplement sensor_msgs/NavSatFix.
std_msgs/Header header

# Measurement status
int16 MODE_NOT_SEEN=0	# mode update not seen yet
int16 MODE_NO_FIX=1	# none
int16 MODE_2D=2	# good for latitude/longitude
int16 MODE_3D=3	# good for altitude/climb too
int16 mode

# STATUS_NO_FIX   0       // no, or unknown
# - yes, plain GPS (SPS Mode), without DGPS, PPS, RTK, DR, etc.
# STATUS_FIX      1
# STATUS_DGPS_FIX 2       /* yes, with DGPS */
# STATUS_RTK_FIX  3       /* yes, with RTK Fixed */
# STATUS_RTK_FLT  4       /* yes, with RTK Float */
# STATUS_DR       5       /* yes, with dead reckoning */
# STATUS_GNSSDR   6       /* yes, with GNSS + dead reckoning */
# STATUS_TIME     7       /* yes, time only (surveyed in, manual) */
# Note that STATUS_SIM and MODE_NO_FIX can go together.
# STATUS_SIM      8       /* yes, simulated */
# - yes, Precise Positioning Service (PPS)
# - Not to be confused with Pulse per Second (PPS)
# - PPS is the encrypted military P(Y)-code
# STATUS_PPS_FIX  9
int16 status

# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude

# Longitude (degrees). Positive is east of prime meridian, negative west.
float64 longitude

# Altitude (meters). Positive is above reference (e.g., sea level).
float64 altitude

# Direction (degrees from north)
float64 track

# Ground speed (meters/second)
float64 speed

# GPS time
float64 time

## Dilution of precision; Xdop<=0 means the value is unknown

# Total (positional-temporal) dilution of precision
float64 gdop

# Positional (3D) dilution of precision
float64 pdop

# Horizontal dilution of precision
float64 hdop

# Vertical dilution of precision
float64 vdop

# Temporal dilution of precision
float64 tdop

## Uncertainty of measurement, 95% confidence

# Spherical position uncertainty (meters) [epe]
float64 err

# Horizontal position uncertainty (meters) [eph]
float64 err_horz

# Vertical position uncertainty (meters) [epv]
float64 err_vert

# Track uncertainty (degrees) [epd]
float64 err_track

# Ground speed uncertainty (meters/second) [eps]
float64 err_speed

# Temporal uncertainty [ept]
float64 err_time

# Number of satellites used in solution
int32 satellites_visible