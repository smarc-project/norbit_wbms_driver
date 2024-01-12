#!/usr/bin/env python3
import socket
import configparser
import os

# Read configuration file and send to Norbit sonar.
# Finally starts the sonar pinging. 
def send_configuration(inifile: str, ip: str, port: str):
    """ 
    Parse an ini file and configure the sonar. 
    ini file: absolute file path
    ip: string
    port: string
    """
    #dir_path = os.path.dirname(os.path.realpath(__file__))
    #print("Current working directory: " + os.getcwd())
    print("Loading config file: " + str(inifile))
    config = configparser.ConfigParser(defaults=None, 
                                    delimiters=(':', '='), 
                                    comment_prefixes=('#', ';'),
                                    inline_comment_prefixes=('#'),
                                    interpolation=None)
    config.read(inifile)

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.settimeout(2)
    print("Connecting to " + ip + " " + port)

    try:
        tcp_socket.connect((ip, int(port)))
        print("TCP successfully connected.")
        print("Sonar welcome message: \n" + str(tcp_socket.recv(1024), 'utf-8'))
    except:
        print("Failed to bind socket.")

    def send(msg: str):
        msg = msg + "\n\r"
        tcp_socket.send(msg.encode())
        print("Sent: " + msg[:-2])
        print("Reply: " + str(tcp_socket.recv(1024), 'utf-8')[:-2])


    # Turn off sonar if string is empty
    if inifile == "":
        send('set_power 0')
        send('exit')
        return

    # Operational Modes
    print('Setting sonar modes')
    sonar = config['sonar']
    modes = config['mode.operation_modes']
    sidescanModes = config['mode.sidescan_modes']
    flipmodes = config['mode.flip_modes']
    gatemodes = config['mode.gate_modes']
    send('set_mode ' + modes.get(sonar['mode']))
    send('set_sidescan_mode ' + sidescanModes.get(sonar['sidescanMode']))
    send('set_flip ' + flipmodes.get(sonar['flipMode']))
    send('set_gate_mode ' + gatemodes.get(sonar['gateMode']))
    send('set_gate_tilt ' + sonar['gateTilt'])
    if sonar['mode'] == 'range':
        send('set_range ' + config['range']['gate_mode_range'])
    elif sonar['mode'] == 'depth':
        send('set_range ' + config['range']['gate_mode_depth'])
    elif sonar['mode'] == 'mixed':
        send('set_range ' + config['range']['gate_mode_range'] + ' ' + config['range']['gate_mode_depth'])

    autogateModes = config['mode.adaptive_gate_modes']
    send('set_autogate_preset ' + autogateModes.get(sonar['autogatePreset']))

    # Swath configuration 
    print('Setting swath parameters')
    swath = config['swath']
    send('set_direction ' + swath['direction'])
    send('set_opening_angle ' + swath['openingAngle'])
    send('set_resolution ' + swath['resolution'])
    send('set_mf_decimation ' + swath['mfDecimation']) # TODO: Really good idea to set this?

    # TODO: Add STX parameters if necessary

    # Transmission Configuration
    print('Setting transmission pulse parameters')
    triggerModes = config['mode.trigger_modes']
    tx = config['tx']
    send('set_tx ' + 
        tx['frequency'] + ' ' + 
        tx['bandwidth'] + ' ' + 
        tx['amplitude'] + ' ' + 
        tx['length'])

    send('set_gain ' + tx['gain'])
    send('set_vga ' + tx['vga'])
    send('set_rate ' + tx['rate'])
    send('set_tp_rate ' + tx['tpRate'])
    send('set_snippet_rate ' + tx['snippetRate'])
    send('set_trigger_mode ' + triggerModes.get(tx['triggerMode']))

    # Environment
    print("Setting environment parameters")
    environment = config['environment']
    timeSources = config['mode.time_source_modes']
    send('sound_velocity ' + environment['soundVelocity'])
    send('set_time_source ' + timeSources.get(environment['timeSource']))
    send('set_ntp_server ' + environment['ntpServer'])

    # Turn on the sonar and close the connection
    send('set_power 1')
    send('exit')
    tcp_socket.close()








