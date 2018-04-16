import requests
import sys,time,struct,argparse
import os

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
        self.measurement_log = "measurements_" + time.strftime("%Y%m%dT%H%M%SZ.txt", time.gmtime())

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

    def configure_database(self):

        try:

            host = os.environ.get('mursu_db_host', 'localhost')
            port = os.environ.get('mursu_db_port', '8086')
            username = os.environ['mursu_db_username']
            password = os.environ['mursu_db_password']
            database = os.environ['mursu_db_database']

            db_client = InfluxDBClient(host=host,port=port,username=username,password=password,database=database)
            self.db_client = db_client

        except KeyError:

            print "Database connection parameters not found - not storing data in database."
            print "Values for this session will be stored only locally in measurements.txt"

        except InfluxDBClientError:

            print "Error connecting to influxdb database at %s %s. Check address and username/password" % (host, port)

    def write_value_to_log(self, measurement):

        with open(self.measurement_log, "ab") as logfile:
            logfile.write(measurement)
            logfile.write("\n")



def create_mursu(address,location):

    mursu_address = address
    device_location = location #"COM3" #"/dev/tty.usbserial-DA00LG9R"
    baudrate = 38400
    timeout = 1
    temperature_register = 1000
    temperature_amount = 6

    mursu_device = MursuServer("Testimittapiste",mursu_address)

    mursu_device.temperature_register = temperature_register
    mursu_device.temperature_amount = temperature_amount

    return mursu_device




def test_local(address):
    mursu_address = int(args.address)
    mursu_device = create_mursu(mursu_address, "test")

    port = mursu.open_and_return_local_mursu_port()
    mursu.configure_database()

    while True:
        measurement = mursu_device.get_temperature(port)
        print "Received measurement:"
        print measurement
        if mursu.db_client:
            mursu.write_value_using_influx_client(float(measurement))


def run_server(address, location, baudrate=38400, timeout=1):

    mursu = create_mursu(address,location)

    try:
        port = mursu.open_port(location,baudrate,timeout)
        mursu.configure_database()

        if port is None:
            print "Opening port failed. Check that port number is correct."
            return

        while True:

            measurement = mursu.get_temperature(port)
            print "Received measurement:"
            print measurement

            if mursu.db_client:

                mursu.write_value_using_influx_client(float(measurement))
                mursu.write_value_to_log(measurement + " " + time.strftime("%Y%m%dT%H%M%SZ.txt", time.gmtime()))

            time.sleep(1)

    except OSError:
        print "Nothing found at %s - check that mursu is connected and uses this port" % device_location


def dbtest():
    mursu_device = create_mursu(1,1)
    mursu_device.configure_database()
    print "Write temperature measurement of 20 deg"
    mursu_device.write_value_using_influx_client(20.0)
    print "Write to local file"
    mursu_device.write_value_to_log(str(20.0))

if __name__ == "__main__":

    # Example: start 1 COM3 where 1 is the address of the mursu and COM3 is the port

    parser = argparse.ArgumentParser(description='Read data from Mursu and store to database')
    subparsers = parser.add_subparsers(title='Available commands',
                                       help='See help for each command with mursu_server.py command_name -h')

    test_parser = subparsers.add_parser('test')
    test_parser.add_argument('address')
    test_parser.set_defaults(func=test_local)

    mursu_parser = subparsers.add_parser('start')
    mursu_parser.add_argument('address')
    mursu_parser.add_argument('location')
    mursu_parser.set_defaults(func=run_server)

    dbtest_parser = subparsers.add_parser('dbtest')
    dbtest_parser.set_defaults(func=dbtest)

    args = parser.parse_args()

    if args.func == dbtest:
        args.func()
    elif args.func == run_server:
        args.func(args.address, args.location)
