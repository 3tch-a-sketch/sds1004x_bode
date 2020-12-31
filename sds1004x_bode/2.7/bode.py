'''
Created on May 5, 2018

@author: dima
'''

import argparse
import sys
from awg_server import AwgServer
from awg_factory import awg_factory

PARSER = argparse.ArgumentParser(
    description=('Emulates a Siglent AWG well enough to support Bode Plots '
        'with a Siglent Oscilliscope and non-Siglent AWG\'s'))

PARSER.add_argument(
    'awg_name',
    help='The name of the AWG driver to use',
    nargs='?',
    default='dummy')

PARSER.add_argument(
    'awg_port',
    help='The AWG port (or device path) to connect to',
    nargs='?',
    default='/dev/ttyUSB0')

PARSER.add_argument(
    'awg_baud_rate',
    help='The baud rate to use',
    nargs='?',
    default='')

PARSER.add_argument(
    '--rpcbind_port',
    help='The RPC bind port to listen on',
    type=int,
    default=111)

PARSER.add_argument(
    '--vxi11_port',
    help='The VXI11 port to listen on',
    type=int,
    default=703)

ARGS = PARSER.parse_args()


if __name__ == '__main__':
    print "Initializing AWG..."
    print '  Driver: %s' % ARGS.awg_name
    print '  Port: %s' % ARGS.awg_port
    if ARGS.awg_baud_rate:
        print '  Baud Rate: %s' % ARGS.awg_baud_rate

    awg_class = awg_factory.get_class_by_name(ARGS.awg_name)
    awg = awg_class(
            ARGS.awg_port, ARGS.awg_baud_rate if ARGS.awg_baud_rate else None)
    awg.initialize()

    server = None
    try:
        server = AwgServer(
                awg,
                rpcbind_port=ARGS.rpcbind_port,
                vxi11_port=ARGS.vxi11_port)
        server.start()
    
    except KeyboardInterrupt:
        print('Ctrl+C pressed. Exiting...')
    
    finally:
        if server is not None:
            server.close_sockets()
    
    print "Bye."
    
