import requests
import sys
import time

import mursu_communications as mursu
from influxdb import InfluxDBClient
from influxdb.exceptions import InfluxDBClientError

class MursuServer():

    def __init__(self,location,address):
        self.temperature_register = 0
        self.temperature_amount = 0
        self.location = location
        self.address = address


    def set_temperature_register_and_amount(self,register,amount):
        self.temperature_register = register
        self.temperature_amount = amount

    def get_temperature(self,port):
        data = mursu.read_holding_register(port,self.address,self.temperature_register,self.temperature_amount)
        mursu.print_response(data)
        temperature = self.parse_temperature(data)
        return temperature 

    def parse_temperature(self,data):
        # Calculate decimal value from data
        #in_c = -46.85 + 175.72 * (data / 2**16)
        #return in_c
        return 20

    def write_value_using_influx(self,value):
        
        client = InfluxDBClient(host='mursuja.rannalle.com', port=8086, username='mursu', password='mursu', database='aavikkomursu')
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
            client.write_points(json_body)
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
    device_location = "/dev/tty.usbserial-DA00LG9R"
    baudrate = 38400
    timeout = 0.1
    temperature_register = 1001
    temperature_amount = 1

    mursu_device = MursuServer("Testimittapiste",mursu_address)
    mursu_device.set_temperature_register_and_amount(temperature_register,temperature_amount)

    if len(sys.argv) > 2 and sys.argv[1] == "test":

        while True:
            if sys.argv[2] == "local":
                port = mursu.open_and_return_local_mursu_port()
            elif sys.argv[2] == "actual":
                try:
                    port = mursu.open_port(device_location,baudrate,timeout)
                except OSError:
                    print "Nothing found at %s - check that mursu is connected and uses this port" % device_location
                    break
            else:
                print "Syntax: mursu_server.py test local | actual"
                break

            measurement = mursu_device.get_temperature(port)
            mursu_device.write_value_using_influx(float(measurement))
            time.sleep(1)
    else:
        print "Syntax: mursu_server.py test local | actual"
        print "Use 'local' when testing without a real device and 'actual' when you have a Mursu connected"
        

