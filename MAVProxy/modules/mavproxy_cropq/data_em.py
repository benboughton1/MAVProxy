import serial
import datetime
import time
import json

import geopy.distance


def nmea_to_json(line):
    split = line.split('*')
    data = split[0].split(',')
    if data[0] == '$PDLMA':
        return data[0], {
            'voltage': float(data[1]),
            'temperature': float(data[2]),
            'pitch': float(data[3]),
            'roll': float(data[4])
        }
    elif data[0][:5] == '$PDLM':
        return data[0], {
            'array_length': str(data[0][5]), # needs to be string as half meter is 'H'
            'time': data[1],
            'hcp_conductivity': float(data[2]),
            'hcp_inphase': float(data[3]),
            'prp_conductivity': float(data[4]),
            'prp_inphase': float(data[5])
        }


def em_connect(self):
    connection = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)
    connection.write(b'%')
    self.em_file = f'em_{datetime.datetime.utcnow().replace(microsecond=0).isoformat()}'

    time_tracker = time.time()

    count = 0

    while True:
        if self.allow_data_collection_threads:
            try:
                if time.time() - time_tracker > 1:
                    connection.write(b'%') # tells em to send extra line with pitch, roll etc
                data = {}
                round_complete = False
                while not round_complete:
                    # print('round not complete', data.keys())
                    line = connection.readline().decode('utf-8')
                    # print(line)
                    if '$' in line:
                        key, json_line = nmea_to_json(line)
                        data[key] = json_line

                    if self.data_col_profiles['em']['model'] == '1s':
                        if '$PDLM1' in data.keys() and '$PDLMA' in data.keys():
                            round_complete = True
                else:
                    # print('round_complete')
                    data['utc_datetime'] = datetime.datetime.utcnow().replace(microsecond=0).isoformat()
                    status_dict = json.loads(self.mpstatus_to_json(self.mpstate.status))
                    gps = status_dict['GPS_RAW_INT']
                    lat = int(gps['lat']) / 1.0e7
                    lon = int(gps['lon']) / 1.0e7
                    data['lat'] = lat
                    data['lon'] = lon
                    data['sats_visible'] = int(gps['satellites_visible'])
                    data['sat_fix'] = int(gps['fix_type'])
                    if int(gps['fix_type']) > 1:
                        geom = f'POINT({str(lon)} {str(lat)})'
                        distance = 0
                        if self.data_col_profiles['em']['last_point']:
                            distance = geopy.distance.geodesic(self.data_col_profiles['em']['last_point'], (lat, lon))
                            print('DISTANCE', distance)
                            if distance > self.data_col_profiles['em']['min_log_distance_m'] / 1000:
                                self.datapoints.append({'dataset': self.data_col_profiles['em']['dataset_id'], 'position': geom, 'data': data})
                                self.data_col_profiles['em']['last_point'] = (lat, lon)
                        else:
                            self.data_col_profiles['em']['last_point'] = (lat, lon)
                            self.datapoints.append({'dataset': self.data_col_profiles['em']['dataset_id'], 'position': geom, 'data': data})
                    f = open(f'/home/pi/datastore/{self.em_file}.txt', 'a')
                    f.write(json.dumps(data) +'\n')
                    f.close()

                count = 0
                    # time.sleep(1)
            except Exception as e:
                print('Problem accessing EM serial - this may resolve itself', count)
                print(e)
                count = count + 1
                if count == 10:
                    print('10 error in row - reconnect')
                    break
            # time.sleep(0.2)
        else:
            break

    connection.close()
    self.em_thread = False
