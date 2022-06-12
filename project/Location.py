import iwlist
import math
import time
import json
import threading


class Location:
    def __init__(self):
        self.stop = False

    def current(self):
        raw = iwlist.scan(interface='wlan0')
        signals = iwlist.parse(raw)
        list = []
        for signal in signals:
            list.append({
                'ssid': signal['essid'],
                'mac': signal['mac'],
                'frequency': float(signal['frequency']),
                'frequency_unit': signal['frequency_units'],
                'signalLevel': int(signal['signal_level_dBm'])
            })
            #print('Distance to ' + signal['essid'] + ': ' + str(self.distance(float(signal['signal_level_dBm']), float(signal['frequency']) * 1000)) + 'm')
        #print(list)
        return list

    def distance(self, signalLevelInDb, freqInMHz):
        exp = (27.55 - (20 * math.log10(freqInMHz)) + abs(signalLevelInDb)) / 20.0
        return math.pow(10.0, exp)

    def thread_function(self, client, sleep_time):
        if self.stop:
            print("stopping location thread...")
            return
        if client is not None:
            client.sendall(json.dumps({'type': 2, 'signals': self.current()}).encode())
        else:
            print(json.dumps(self.current()))
        time.sleep(sleep_time)
        self.thread_function(client, sleep_time)


if __name__ == "__main__":
    locationClass = Location()
    location_thread = threading.Thread(target=locationClass.thread_function, args=(None, 1))
    try:
        location_thread.start()
        time.sleep(100)
    except:
        print('cleaning up...')
        locationClass.stop = True
        location_thread.join()
