import re
import numpy as np

def read_log_dyn_press(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" AIRSPEED_MS45XX (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_SDP3X(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" AIRSPEED_SDP3X (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_aoa_flags(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" AOA (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])
    return np.array(list_meas)
    

def read_log_aoa_press(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" PAYLOAD_FLOAT (\S+),(\S+),(\S+),(\S+),(\S+),(\S+),(\S+),(\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7)), float(m.group(8)), float(m.group(9))])
    return np.array(list_meas)

def read_log_desired(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" DESIRED (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7)), float(m.group(8)), float(m.group(9))])
    return np.array(list_meas)

def read_log_payload4(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" PAYLOAD_FLOAT (\S+),(\S+),(\S+),(\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))])
    return np.array(list_meas)
    
def read_log_gps(ac_id, filename):
    """Extracts gps values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" GPS (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), 
           float(m.group(7)), float(m.group(8)), float(m.group(9)), float(m.group(10)), float(m.group(11)),float(m.group(12))])
    return np.array(list_meas)

def read_log_gps_int(ac_id, filename):
    """Extracts gps values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" GPS_INT (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), 
           float(m.group(7)), float(m.group(8)), float(m.group(9)), float(m.group(10)), float(m.group(11)),float(m.group(12)), float(m.group(13)),
           float(m.group(14)), float(m.group(15)), float(m.group(16)),float(m.group(17)),float(m.group(18))])
    return np.array(list_meas)    
    
def read_log_attitude(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" ATTITUDE (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)
    
def read_log_actuators(ac_id, filename):
    """Extracts ACTUATOR values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" ACTUATORS (\S+),(\S+),(\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_commands(ac_id, filename):
    """Extracts ACTUATOR values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" COMMANDS (\S+),(\S+),(\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)
    
def read_log_energy(ac_id, filename):
    """Extracts Energy sensor values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" ENERGY (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))])
    return np.array(list_meas)

def read_log_energy_new(ac_id, filename):
    """Extracts New Energy sensor values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" ENERGY (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7))])
    return np.array(list_meas)

def read_log_air_data(ac_id, filename):
    """Extracts Air-data values from a log.  Ps, Pd, temp,qnh, amsl_baro, airspeed, TAS"""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" AIR_DATA (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7)), float(m.group(8))])
    return np.array(list_meas)

# wx wz Va gamma AoA Theta_commanded

def read_log_gust(ac_id, filename):
    """Extracts Energy sensor values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" GUST (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7))])
    return np.array(list_meas) 

def read_log_imuaccel(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_ACCEL (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_imuaccel_scaled(ac_id, filename):
    """Extracts generic adc values from a log."""
    k = 0.0009766 # imu_accel_scaled coeff
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_ACCEL_SCALED (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2))*k, float(m.group(3))*k, float(m.group(4))*k])
    return np.array(list_meas)

def read_log_imuaccel_raw(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_ACCEL_RAW (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_imugyro(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_GYRO (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_imugyro_scaled(ac_id, filename):
    """Extracts generic adc values from a log."""
    k = 0.0139882*0.017453292519943295 # imu_gyro_scaled coeff
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_GYRO_SCALED (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2))*k, float(m.group(3))*k, float(m.group(4))*k])
    return np.array(list_meas)

def read_log_imugyro_raw(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_GYRO_RAW (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)

def read_log_fault_telemetry(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" FAULT_TELEMETRY (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2))])
    return np.array(list_meas)

def read_log_mode(ac_id, filename):
    """Extracts mode values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" PPRZ_MODE (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7))])
    return np.array(list_meas)

def read_log_settings(ac_id, filename):
    """Extracts generic adc values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" SETTINGS (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)),  float(m.group(5))])
    return np.array(list_meas)

  # <message name="ROTORCRAFT_FP" id="147">
  #   <field name="east"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="north"    type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="up"       type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="veast"    type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
  #   <field name="vnorth"   type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
  #   <field name="vup"      type="int32" alt_unit="m/s" alt_unit_coef="0.0000019"/>
  #   <field name="phi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
  #   <field name="theta"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
  #   <field name="psi"      type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
  #   <field name="carrot_east"   type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="carrot_north"  type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="carrot_up"     type="int32" alt_unit="m" alt_unit_coef="0.0039063"/>
  #   <field name="carrot_psi"    type="int32" alt_unit="deg" alt_unit_coef="0.0139882"/>
  #   <field name="thrust"        type="int32"/>
  #   <field name="flight_time"   type="uint16" unit="s"/>
  # </message>

def read_log_rotorcraft_fp(ac_id, filename):
    """Extracts rotorcraft_fp values from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" ROTORCRAFT_FP (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), 
           float(m.group(7)), float(m.group(8)), float(m.group(9)), float(m.group(10)), float(m.group(11)),float(m.group(12)), 
           float(m.group(13)), float(m.group(14)), float(m.group(15)),float(m.group(16)) ])
    return np.array(list_meas)


# <message name="SOARING_TELEMETRY" id="212">
# <field name="velocity"     type="float"  unit="m/s">veocity</field>
# <field name="a_attack"     type="float"  unit="rad">angle of attack</field>
# <field name="a_sideslip"   type="float"  unit="rad">sideslip angle</field>
# <field name="dynamic_p"    type="float"  unit="Pa"/>
# <field name="static_p"     type="float"  unit="Pa"/>
# <field name="wind_x"       type="float"  unit="m/s"/>
# <field name="wind_z"       type="float"  unit="m/s"/>
# <field name="wind_x_dot"   type="float"  unit="m/s2"/>
# <field name="wind_z_dot"   type="float"  unit="m/s2"/>
# <field name="wind_power"   type="float"  unit="W"/>

def read_log_soaring_telemetry(ac_id, filename):
    """Extracts Air-data values from a log.  Ps, Pd, temp,qnh, amsl_baro, airspeed, TAS"""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" SOARING_TELEMETRY (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
           list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5)), float(m.group(6)), float(m.group(7)), float(m.group(8)), float(m.group(9)), float(m.group(10)), float(m.group(11))])
    return np.array(list_meas)
