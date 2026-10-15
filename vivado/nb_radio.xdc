# nb_radio: RFDC analog/clock pins are dedicated (no LOC needed); RFDC + clk_wiz generate the
# clock constraints. Only the clock-domain crossings need attention:
#  * PS 100 MHz <-> radio 320 MHz goes through SmartConnect and async axis_data_fifo IPs,
#    which carry their own CDC constraints.
#  * The modem's register outputs are quasi-static (written via SmartConnect in the radio
#    domain), so no extra exceptions are required.
