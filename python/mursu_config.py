import serial
import mursu_communications as mursu
import argparse, sys, time

#com = serial.Serial( 6, 38400, timeout = 0.1 )



def read(args):
    print("Reading %s register(s) starting from register %s of device %s" %
          (args.amount,args.address, args.register))
    port = mursu.open_port()
    response = mursu.read_holding_register(port,args.address,args.register,
                                       args.amount)
    mursu.print_response(response)
    mursu.close(port)
    

def write(args):
    print("Writing single register %s of device %s" % (args.address, args.register))
    port = mursu.open_port()
    response = mursu_server.write_single_register(port,args.address,args.register,
                                       args.amount)
    mursu.print_response(response)
    mursu.close(port)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Configure Aavikkomursu')
    subparsers = parser.add_subparsers(title='Available commands',
                                       help='See help for each command with mursu_config.py command_name -h')

    read_parser = subparsers.add_parser('read')
    read_parser.add_argument('address')
    read_parser.add_argument('register')
    read_parser.add_argument('amount')
    read_parser.set_defaults(func=read)

    write_parser = subparsers.add_parser('write')
    write_parser.add_argument('address')
    write_parser.add_argument('register')
    write_parser.add_argument('amount')
    write_parser.set_defaults(func=write)
    
    args = parser.parse_args()
    args.func(args)

