#!/usr/bin/python3
import json
import select
import socket
import threading
import sys
from uuid import getnode as get_mac
MAC_ADDRESS = ':'.join(("%012X" % get_mac())[i:i+2] for i in range(0, 12, 2))

from RBGctrl import RGBctrl
from Engine import Engine
from Location import Location

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.setblocking(1)
client.connect(('spikeone.goip.de', 3665))

client.send(('{"type":1,"currentPosition":{"x":1,"y":1.0,"z":1.1,"o":1.1,"floor":1},"robotName":"Rob","mac":"' + MAC_ADDRESS + '"}').encode())
# client.send(('{"type":1,"currentPosition":{"x":1.1,"y":1.1,"z":1.1,"o":1.1,"floor":1},"robotName":"Rob","macAddress":"MM:MM:MM:SS:SS:SS"}').encode())

print("connected...")

RGBCtrl_1 = RGBctrl()
RGBCtrl_1.setup()
engine = Engine()
engine.setup()


def color_light(command):
    side = command["side"]
    r = command["color"]["r"]
    g = command["color"]["g"]
    b = command["color"]["b"]
    if side == 0:
        RGBCtrl_1.set_left_color(r, g, b)
        RGBCtrl_1.set_right_color(r, g, b)
    elif side == 1:
        RGBCtrl_1.set_left_color(r, g, b)
    elif side == 2:
        RGBCtrl_1.set_right_color(r, g, b)
    else:
        print("invalid side: " + str(side))


def color_off(command):
    side = command["side"]
    turn_on = command["state"]
    if side == 0:
        if turn_on:
            RGBCtrl_1.set_left_color(1, 1, 1)
            RGBCtrl_1.set_right_color(1, 1, 1)
        else:
            RGBCtrl_1.set_left_off()
            RGBCtrl_1.set_right_off()
    elif side == 1:
        if turn_on:
            RGBCtrl_1.set_left_color(1, 1, 1)
        else:
            RGBCtrl_1.set_left_off()
    elif side == 2:
        if turn_on:
            RGBCtrl_1.set_right_color(1, 1, 1)
        else:
            RGBCtrl_1.set_right_off()
    else:
        print("invalid side: " + str(side))


def engine_command(command):
    direction = command["direction"]
    if direction == 1:
        engine.motorForward(100)
    elif direction == 0:
        engine.motorStop()
    elif direction == 2:
        engine.motorBackward(100)
    # engine.move(100, 'forward', 1, None)
    # {'angle': 0, 'direction': 1, 'type': 5}


commands: dict = {1: color_off, 2: color_light, 5: engine_command}
locationClass = Location()


location_thread = threading.Thread(target=locationClass.thread_function, args=(client, 1))
location_thread.start()

try:
    while True:
        # maintains a list of possible input streams
        sockets_list = [sys.stdin, client]
        read_sockets, write_socket, error_socket = select.select(sockets_list, [], [])
        for socks in read_sockets:
            if socks == client:
                message = socks.recv(2048)
                command = message.decode()
                command_dict = json.loads(command)
                print(command_dict)
                dict_key = command_dict["type"]
                if dict_key in commands:
                    commands[dict_key](command_dict)
                else:
                    print("Unknown command: " + str(dict_key))
                    client.send('{"type":0, "message":"unknown command"}'.encode())
            else:
                message = sys.stdin.readline()
                client.send(message.encode())
                sys.stdout.write("<You>")
                sys.stdout.write(message)
                sys.stdout.flush()
except KeyboardInterrupt or ConnectionResetError or KeyError or BrokenPipeError or json.decoder.JSONDecodeError:
    print('cleaning up...')
    locationClass.stop = True
    location_thread.join()
    client.close()
    RGBCtrl_1.destroy()
