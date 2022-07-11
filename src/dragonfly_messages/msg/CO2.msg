# Status/Error code
int8 NO_ERROR      = 0 # No errors
int8 LOW_COUNT     = 1 # Less than 25000 counts
int8 TOO_COLD      = 2 # Less than 5°C from user specified temperature
int8 TOO_HOT       = 3 # Greater than 5°C from user specified temperature
int8 LOW_ALARM     = 4 # less than range from L command
int8 HIGH_HUMIDITY = 5 # greater than 90 mbar
int8 LOW_VOLTAGE   = 6 # 6 Board voltage less than 4V

# Set the ambient ppm
# use in python insted of magic numbers
# from dragonfly_messages.msg import CO2
# CO2.ambient_ppm # this should yeild the int below
# CO2.threshold_ppm # this should yeild the int below
float32 AMBIENT_PPM         = 419
float32 PLUME_THRESHOLD_PPM = 900 #??????? 

int32 zero_count
int32 count
float32 ppm
float32 average_temp # in c
float32 humidity
float32 humidity_sensor_temp
int32 atmospheric_pressure #Atmospheric pressure in IRGA [mbar],
float32 detector_temp #hh.h IRGA detector temperature [°C],
float32 source_temp #IRGA source temperature [°C],
int8 status #Status/Error code. Continuously displayed measurements do not display the j but instead display a text message
