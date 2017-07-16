from pymavlink import mavutil
import serial
import sys
import glob
import warnings
import serial.tools.list_ports

def main():
    device = "127.0.0.1:14549"

    proxy_mavlink(device)

def findAndConfigureArduinoPort():

    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'A904OU84' in p.serial_number
        ]

    if not arduino_ports:
        raise IOError("No Antenna Tracker found")

    return serial.Serial(port=arduino_ports[0],
                        baudrate=115200,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS
    )

def proxy_mavlink(device):
    """Receives packets over the device and forwards telemetry via the client.

    Args:
        device: A pymavlink device name to forward.
    """
    # Create the MAVLink connection.
    mav = mavutil.mavlink_connection(device, autoreconnect=True)

    ser = findAndConfigureArduinoPort()

    #ser.open()

    # Continuously forward packets.
    while True:
        # Get packet.
        msg = mav.recv_match(type='GLOBAL_POSITION_INT',
                             blocking=True,
                             timeout=5)
        if msg is None:
            print "Did not receive MAVLink packet for over 5 seconds."

        if msg is not None:
            ser.write(("{},{},{}\n".format(msg.lat/1e7, msg.lon/1e7, msg.alt/1e3)))

            log = open("tracker_data_log2.txt", "a")
            log.write(("{},{},{}\n".format(msg.lat/1e7, msg.lon/1e7, msg.alt/1e3)))
            log.close()

        #print ("{},{},{}".format(msg.lat, msg.lon, msg.alt))

if __name__ == '__main__':
    main()
