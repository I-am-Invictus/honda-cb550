


class charging_control:
    def __init__(self):
        # Constants (example)
        self.NUM_CELLS = 20
        self.CELL_MAX_VOLTAGE = 4.2
        self.PACK_MAX_VOLTAGE = self.NUM_CELLS * self.CELL_MAX_VOLTAGE  # 100.8V
        self.CHARGE_CURRENT_CC = 10.0  # Amps (0.5C for 20Ah pack)
        self.CHARGE_CURRENT_CUTOFF = 1.0  # Amps (0.05C)

        # Variables (to be read from sensors)
        self.pack_voltage = 0.0
        self.charge_current = 0.0

        self.charging = False
        self.stage = 'CC'  # Start with constant current
    
    def start_charging(self, pack_voltage, charge_current):
        self.charging = True
        self.run_update(pack_voltage, charge_current)

    def stop_charging(self):
        self.charging = False
        self.pack_voltage = 0.0
        self.charge_current = 0.0

    def run_update(self, pack_voltage, charge_current):
        output_charge_current = 0.0
        output_charge_voltage = 0.0

        if self.charging:
            if stage == 'CC':
                output_charge_voltage = self.PACK_MAX_VOLTAGE + .2
                output_charge_current = self.CHARGE_CURRENT_CC
                if pack_voltage >= self.PACK_MAX_VOLTAGE:
                    stage = 'CV'  # Switch to constant voltage

            elif stage == 'CV':
                output_charge_voltage = self.PACK_MAX_VOLTAGE
                output_charge_current = self.CHARGE_CURRENT_CC + 5
                if charge_current <= self.CHARGE_CURRENT_CUTOFF:
                    self.stop_charging()
                    output_charge_current = 0.0
                    output_charge_voltage = 0.0

        return output_charge_current, output_charge_voltage
