import requests
import sys
import time

import mursu_communications as mursu
from influxdb import InfluxDBClient
from influxdb.exceptions import InfluxDBClientError

class MursuServer():

    def __init__(self,location,debug=False,database_address="http://mursuja.rannalle.com:8086",t_register = 1000,t_amount = 1):
        self.temperature_register = t_register
        self.temperature_amount = t_amount
        self.location = location
        self.database_address = database_address
        self.port = 6
        self.baudrate = 38400
        self.timeout = 0.1
        self.debug = debug

    def get_temperature(self,address,port):
        data = mursu.read_holding_register(port,address,self.temperature_register,self.temperature_amount)
        mursu.print_message(data)
        temperature = self.parse_temperature(data)
        return temperature 

    def parse_temperature(self,data):
        # Calculate decimal value from data
        #in_c = -46.85 + 175.72 * (data / 2**16)
        #return in_c
        return 20

    def write_measurement_to_db(self,measurement):
        temperature = parse_temperature(measurement)
        url = self.database_address + "/write?db=" + "aavikkomursu&u=mursu&p=mursu"
        payload = "measurement,location=" + self.location + " value=" + str(temperature)
        #binary_data = payload.encode('utf-8')
        req = requests.post(url=url,data=binary_data)
        if req.status_code != 204:
            print("Error posting data, response content was: ")
            print(req.text)


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

    def write_simple_value(self,value):
        url = self.database_address + "/write?db=" + "aavikkomursu&u=mursu&p=mursu"
        payload_string = "measurement,location=" + self.location + " value=" + str(value)
        payload = {'string': payload_string}
        binary_data = payload_string.encode('utf-8')
        print binary_data
        print type(binary_data)
        #print type(b'a')
        req = requests.post(url=url,data=binary_data)
        print req.url
        print req.status_code
        print req.text
        if req.status_code != 204:
            print("Error posting data, response content was: ")
            print(req.text)

    def query_database(self):
        url = self.database_address + "/query?"
        parameters = {'db' : 'aavikkomursu','q':'SELECT * from temperature'}
        request = requests.get(url=url,params = parameters)
        print request.text
        print request.status_code
        print request.url

    def configure_database(self,new_address):
        self.database_address = new_address

    def configure_serial(self,port,baudrate,timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

if __name__ == "__main__":

    m = MursuServer("Testimittapiste",True)

    if len(sys.argv) > 1 and sys.argv[1] == "test":
        while True:
            #m.query_database()
            port = mursu.open_and_return_local_mursu_port()
            measurement = m.get_temperature(250,port)
            m.write_value_using_influx(float(measurement))
            time.sleep(1)


    else:
        port = mursu.open_port(self.port,self.baudrate,self.timeout)

        address = 250

        while True():
            try:
                measurement = m.get_temperature(address,port)
                m.write_to_db(measurement)
            except:
                "An error happened.."
            finally:
                mursu.close(port)
        

