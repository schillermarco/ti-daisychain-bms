number_of_chains = 5
number_of_slaves = 16
number_of_cells = 16


dbc_file_voltages = open("voltages.txt", "w")
dbc_file_temperatures_battery = open("temperatures_battery.txt", "w")
dbc_file_temperatures_pcb = open("temperatures_pcb.txt", "w")


for chain in range(number_of_chains):
    for slave in range(number_of_slaves):
        for cell in range(number_of_cells):
            chain_string = str(chain).rjust(2, "0")
            slave_string = str(slave).rjust(2, "0")
            cell_string = str(cell).rjust(2, "0")
            mux_id = (chain<<16) + (slave << 8) + cell

            dbc_string_voltage = f" SG_ PICO_BMS_FB_voltage_{chain_string}_{slave_string}_{cell_string}  m{mux_id}: 31|16@0- (0.001,0) [0|5] \"V\" Vector__XXX\n"
            dbc_file_voltages.write(dbc_string_voltage)

            dbc_string_temperature_battery = f" SG_ PICO_BMS_FB_temperature_battery_{chain_string}_{slave_string}_{cell_string}  m{mux_id}: 31|16@0- (0.1,0) [0|100] \"C\" Vector__XXX\n"
            dbc_file_temperatures_battery.write(dbc_string_temperature_battery)

            dbc_string_temperature_pcb = f" SG_ PICO_BMS_FB_temperature_pcb_{chain_string}_{slave_string}_{cell_string}  m{mux_id}: 31|16@0- (0.1,0) [0|100] \"C\" Vector__XXX\n"
            dbc_file_temperatures_pcb.write(dbc_string_temperature_pcb)


dbc_file_voltages.close()
dbc_file_temperatures_battery.close()
dbc_file_temperatures_pcb.close()
