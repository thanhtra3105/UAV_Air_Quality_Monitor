import time
import struct
from smbus2 import SMBus

# I2C addresses
ENS160_ADDR = 0x53
AHT21_ADDR = 0x38

# ENS160 Registers
ENS160_PART_ID = 0x00
ENS160_OPMODE = 0x10
ENS160_DATA_AQI = 0x20
ENS160_DATA_ECO2 = 0x22
ENS160_DATA_TVOC = 0x24
ENS160_DATA_STATUS = 0x20
ENS160_TEMP_IN = 0x2A
ENS160_RH_IN = 0x2C

# AHT21 Commands
AHT21_CMD_INIT = [0xBE, 0x08, 0x00]
AHT21_CMD_MEASURE = [0xAC, 0x33, 0x00]

# ENS160 Operating Modes
OPMODE_DEEP_SLEEP = 0x00
OPMODE_IDLE = 0x01
OPMODE_STANDARD = 0x02
OPMODE_RESET = 0xF0

class ENS160:
    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        self.addr = ENS160_ADDR
        
    def write_register(self, reg, value):
        """Write single byte to register"""
        self.bus.write_byte_data(self.addr, reg, value)
        
    def write_register_word(self, reg, value):
        """Write 16-bit value to register"""
        self.bus.write_word_data(self.addr, reg, value)
        
    def read_register(self, reg, length=1):
        """Read bytes from register"""
        return self.bus.read_i2c_block_data(self.addr, reg, length)
    
    def read_register_word(self, reg):
        """Read 16-bit value from register"""
        return self.bus.read_word_data(self.addr, reg)
    
    def init(self):
        """Initialize ENS160 sensor"""
        # Check part ID
        part_id = self.read_register_word(ENS160_PART_ID)
        print(f"ENS160 Part ID: 0x{part_id:04X}")
        
        if part_id != 0x0160:
            print("Warning: Unexpected Part ID")
            
        # Reset sensor
        self.write_register(ENS160_OPMODE, OPMODE_RESET)
        time.sleep(0.05)
        
        # Set to standard mode
        self.write_register(ENS160_OPMODE, OPMODE_STANDARD)
        time.sleep(0.05)
        print("ENS160 initialized in STANDARD mode")
        
    def set_compensation(self, temperature, humidity):
        """Set temperature and humidity compensation"""
        # Temperature in Celsius (convert to Kelvin * 100)
        temp_k = (temperature + 273.15) * 100
        self.write_register_word(ENS160_TEMP_IN, int(temp_k))
        
        # Humidity in %RH * 100
        self.write_register_word(ENS160_RH_IN, int(humidity * 100))
        
    def read_data(self):
        """Read air quality data"""
        # Read status
        status = self.read_register(ENS160_DATA_STATUS, 1)[0]
        
        if status & 0x01:  # New data available
            # Read AQI, eCO2, TVOC
            data = self.read_register(ENS160_DATA_AQI, 6)
            
            aqi = data[0]
            eco2 = data[1] | (data[2] << 8)
            tvoc = data[3] | (data[4] << 8)
            
            return {
                'aqi': aqi,
                'eco2': eco2,
                'tvoc': tvoc,
                'valid': True
            }
        else:
            return {'valid': False}

class AHT21:
    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        self.addr = AHT21_ADDR
        
    def write_command(self, cmd):
        """Write command to sensor"""
        self.bus.write_i2c_block_data(self.addr, cmd[0], cmd[1:])
        
    def read_data(self, length):
        """Read data from sensor"""
        return self.bus.read_i2c_block_data(self.addr, 0x00, length)
    
    def init(self):
        """Initialize AHT21 sensor"""
        try:
            self.write_command(AHT21_CMD_INIT)
            time.sleep(0.04)
            print("AHT21 initialized")
            return True
        except Exception as e:
            print(f"AHT21 init error: {e}")
            return False
    
    def measure(self):
        """Measure temperature and humidity"""
        try:
            # Start measurement
            self.write_command(AHT21_CMD_MEASURE)
            time.sleep(0.08)  # Wait for measurement
            
            # Read data
            data = self.read_data(6)
            
            if len(data) == 6:
                # Parse humidity
                humidity_raw = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
                humidity = (humidity_raw / 1048576.0) * 100
                
                # Parse temperature
                temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
                temperature = (temp_raw / 1048576.0) * 200 - 50
                
                return {
                    'temperature': temperature,
                    'humidity': humidity,
                    'valid': True
                }
            else:
                return {'valid': False}
                
        except Exception as e:
            print(f"AHT21 measure error: {e}")
            return {'valid': False}

def detect_sensors(bus=1):
    """Detect I2C devices"""
    smbus = SMBus(bus)
    devices = []
    
    for addr in range(0x03, 0x78):
        try:
            smbus.write_byte(addr, 0x00)
            devices.append(hex(addr))
        except:
            pass
            
    smbus.close()
    return devices

def main():
    print("Initializing sensors...")
    
    # Detect I2C devices
    print(f"Detected devices: {detect_sensors()}")
    
    # Initialize sensors
    ens160 = ENS160()
    aht21 = AHT21()
    
    # Initialize AHT21
    if not aht21.init():
        print("Failed to initialize AHT21")
        return
        
    # Initialize ENS160
    try:
        ens160.init()
    except Exception as e:
        print(f"Failed to initialize ENS160: {e}")
        return
    
    print("\n" + "="*60)
    print("Starting sensor readings (press Ctrl+C to stop)")
    print("="*60)
    
    try:
        while True:
            # Read from AHT21
            aht_data = aht21.measure()
            
            if aht_data['valid']:
                temp = aht_data['temperature']
                humidity = aht_data['humidity']
                
                # Set compensation for ENS160
                ens160.set_compensation(temp, humidity)
                time.sleep(0.05)  # Wait for compensation
                
                # Read from ENS160
                ens_data = ens160.read_data()
                
                # Display results
                print("\n" + "-"*50)
                print(f"Temperature: {temp:.2f} °C")
                print(f"Humidity: {humidity:.2f} %")
                
                if ens_data['valid']:
                    aqi = ens_data['aqi']
                    eco2 = ens_data['eco2']
                    tvoc = ens_data['tvoc']
                    
                    # AQI Interpretation
                    aqi_desc = {
                        1: "Excellent",
                        2: "Good",
                        3: "Moderate", 
                        4: "Poor",
                        5: "Unhealthy"
                    }.get(aqi, "Unknown")
                    
                    print(f"\n--- Air Quality ---")
                    print(f"AQI: {aqi} ({aqi_desc})")
                    print(f"eCO2: {eco2} ppm")
                    print(f"TVOC: {tvoc} ppb")
                    
                    if eco2 > 1000:
                        print("⚠️  High CO2 levels, ventilate area!")
                    if tvoc > 500:
                        print("⚠️  High VOC levels detected!")
                else:
                    print("Waiting for ENS160 data...")
                
                print("-"*50)
                
            else:
                print("Failed to read AHT21 data")
            
            time.sleep(2)  # Read every 2 seconds
            
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()