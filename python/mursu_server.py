import Requests
import mursu_communications as mursu

def get_temperature():
    pass

def parse_temperature(data):
    # Calculate decimal value from data
    in_c = -46.85 + 175.72 * (data / 2**16)
    return in_c

def write_to_db(measurement):
    payload = 
    req = Requests.    
