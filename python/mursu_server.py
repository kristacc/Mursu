import requests
import mursu_communications as mursu

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
        temperature = parse_temperature(data)
        return temperature 

    def parse_temperature(self,data):
        # Calculate decimal value from data
        in_c = -46.85 + 175.72 * (data / 2**16)
        return in_c

    def write_measurement_to_db(self,measurement):
        temperature = parse_temperature(measurement)
        url = self.database_address + "/write?db=" + "aavikkomursu&u=mursu&p=mursu"
        payload = "measurement,location=" + self.location + " value=" + str(temperature)
        #binary_data = payload.encode('utf-8')
        req = requests.post(url=url,data=binary_data)
        if req.status_code != 204:
            print("Error posting data, response content was: ")
            print(req.text)
                                         		

    def write_simple_value(self,value):
        url = self.database_address + "/write?db=" + "aavikkomursu&u=mursu&p=mursu"
        payload = "measurement,location=" + self.location + " value=" + str(value)
        #binary_data = payload.encode('utf-8')
        req = requests.post(url=url,data=payload)
        print req
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

    if m.debug == False:
        port = mursu.open_port(self.port,self.baudrate,self.timeout)

        address = 100

        while True():
            try:
                measurement = m.get_temperature(port,address)
                m.write_to_db(measurement)
            except:
                "An error happened.."
            finally:
                mursu.close(port)
    else:
        # for debug purposes,
        # post a value to database
        mursu.query_database()
        mursu.write_simple_value(2)

