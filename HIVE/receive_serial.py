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
    # print('pc receives: '+line+'\n')
    print(line)
    return

def echo(ser):

    msg = input('pc to duino: ')
    print('PC sends: ' + msg)
    send_msg(ser,msg)
    receive_msg(ser)

    return


if __name__ == '__main__':

    # to see active ports in cmd,
    # 1) open cmd
    # 2) type "powershell"
    # 3) type "[System.IO.Ports.SerialPort]::getportnames()"

    #ser = initialize_com(9600);
    ser = initialize_com(38400);

    # print("omega_l_des [rad/s], omega_l [rad/s], error_l [rad/s], u_integral_l , u_l [8bit], omega_r_des [rad/s], omega_r [rad/s], error_r [rad/s], u_integral_r , u_r [8bit]");

    #msg = input('pc to duino: ')
    #send_msg(ser,msg)

    while True:
        # case 1 - only receive
        receive_msg(ser)
        time.sleep(0.1)

        # case 2 - send and receive
        #echo(ser)
        #time.sleep(1)

        # case 3 - only send
        #msg = input('pc to duino: ')
        #send_msg(ser,msg)
