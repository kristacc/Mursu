import requests
import sys
import time
import struct

import mursu_communications as mursu
from influxdb import InfluxDBClient
from influxdb.exceptions import InfluxDBClientError

class MursuServer():

    def __init__(self,location,address):
        self.temperature_register = 0
        self.temperature_amount = 0
        self.location = location
        self.address = address
        self.db_client = None

    def get_temperature(self,port):
        data = mursu.read_holding_register(port,self.address,self.temperature_register,self.temperature_amount)
        temperature = self.parse_temperature(data)
        return temperature 

    def parse_temperature(self,data):
        
        temp_value = data[3:-2]
        print len(temp_value)
        (t0,p,rh,t1) = struct.unpack_from(">IIHH",temp_value,0)
        t0 /= 100.0
        return t0

    def write_value_using_influx_client(self,value):
        
        try:
            json_body = [
            {
                "measurement": "temperature",
                "tags": {
                    "host": "server01",
                    "region": "us-west"
                },
                #"time": "2009-11-10T23:00:00Z",
                "fields": {
                    "value": value
                }
            }
            ]
            self.db_client.write_points(json_body)
        except InfluxDBClientError:
            print "Value needs to be a float"

    def query_database(self):
        url = self.database_address + "/query?"
        parameters = {'db' : 'aavikkomursu','q':'SELECT * from temperature'}
        request = requests.get(url=url,params = parameters)
        print request.text
        print request.status_code
        print request.url


if __name__ == "__main__":

    mursu_address = 100
    device_location = 2 #"COM3" #"/dev/tty.usbserial-DA00LG9R"
    baudrate = 38400
    timeout = 1
    temperature_register = 1000
    temperature_amount = 6

    client = InfluxDBClient()

    mursu_device = MursuServer("Testimittapiste",mursu_address)

    mursu_device.temperature_register = temperature_register
    mursu_device.temperature_amount = temperature_amount
    mursu_device.db_client = client

    if len(sys.argv) > 2 and sys.argv[1] == "test":

        
        if sys.argv[2] == "local":
            port = mursu.open_and_return_local_mursu_port()
        elif sys.argv[2] == "actual":
            try:
                port = mursu.open_port(device_location,baudrate,timeout)
                while True:
                    measurement = mursu_device.get_temperature(port)
                    print "Got measurement:"
                    print measurement
                    #mursu_device.write_value_using_influx_client(float(measurement))
                    time.sleep(1)
            except OSError:
                print "Nothing found at %s - check that mursu is connected and uses this port" % device_location
               
        else:
            print "Syntax: mursu_server.py test local | actual"
                
        

    else:
        print "Syntax: mursu_server.py test local | actual"
        print "Use 'local' when testing without a real device and 'actual' when you have a Mursu connected"
        

