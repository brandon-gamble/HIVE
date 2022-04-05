from serial import Serial
from serial.tools.list_ports import comports
import time

def initialize_com(baud):

    active_ports = comports()
    # if there is only one com port, use that one
    if (len(active_ports) == 1):
        port_id = active_ports[0].name
    # if there are multiple com ports, ask user which to use
    else:
        print('Active ports: ')
        for port in active_ports:
            print(' - ' + port.name)
        port_id = input('What port would you like to use? ')
    print('Using port: ' + port_id)

    ser = Serial(port_id, baud, timeout=1)
    ser.flush()

    return ser

def send_msg(ser, msg):
    ser.write((msg+'\n').encode('utf-8'))
    return

def receive_msg(ser):
    line = ser.readline().decode('utf-8').rstrip()
    print('pc receives: '+line+'\n')
    return

if __name__ == '__main__':

    # to see active ports in cmd,
    # 1) open cmd
    # 2) type "powershell"
    # 3) type "[System.IO.Ports.SerialPort]::getportnames()"

    '''
    1 - manual type message to send
    2 - auto send test messages
    '''
    case = 2

    ser = initialize_com(38400);
    print('waiting for connection....')
    time.sleep(5)
    print('***************************')
    print('COMMANDS:')
    print('L, left')
    print('R, right')
    print('S, standby')

    print('FORMAT:')
    print('<L, 100>')
    print('***************************')

    if case == 1 :
        while True:
            msg = input('actuator command:')
            send_msg(ser,msg)

    elif case == 2:
        while True:
            print('******************************************************')
            print('*               beginning test sequence              *')
            print('******************************************************')

            send_msg(ser,'<S,0>')
            print('motors enabled')

            send_msg(ser,'<L, 100>')
            send_msg(ser,'<R, 100>')
            print('both motors drive @ 100')
            print('***************************')
            time.sleep(3);

            send_msg(ser,'xx<L, -50>')
            send_msg(ser,'blah')
            send_msg(ser,'<R, -50>xx')
            print('both motors drive @ -50, ignoring leading, intermediate, and trailing serial buffer')
            print('***************************')
            time.sleep(3);

            send_msg(ser,'<B>')
            print('both motors brake')
            print('***************************')
            time.sleep(2)

            send_msg(ser,'<L, ')
            send_msg(ser,' 200>')
            send_msg(ser,'<R')
            send_msg(ser,', -200>')
            print('L @ 200, R @ -200, command split between sends')
            print('***************************')
            time.sleep(2);

            send_msg(ser,'<L, -200> <R, -200>')
            print('both @ -200, sent in one msg')
            time.sleep(2);
            print('***************************')

            send_msg(ser,'<S, 1>')
            print('both motors on standby')
            print('***************************')
            time.sleep(5)
