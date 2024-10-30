std_msgs/Header header
# battery pack current in amps. defined as (load current – charger current): negative pack current means the chargers
# are charging more than the load is drawing, so the pack is being charged
float32 pack_current
# load current in amps
float32 load_current
# battery pack voltage in volts (always positive)
float32 pack_voltage
# SOC percent
float32 soc_percent