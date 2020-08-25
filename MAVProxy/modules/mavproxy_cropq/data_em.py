import serial
import datetime
import time
import json

import geopy.distance


def nmea_to_json(line):
    split = line.split('*')
    data = split[0].split(',')
    if data[0] == '$PDLMA':
        return {
            'voltage': float(data[1]),
            'temperature': float(data[2]),
            'pitch': float(data[3]),
            'roll': float(data[4])
        }
    elif data[0][:5] == '$PDLM':
        return {
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
        try:
            if time.time() - time_tracker > 1:
                connection.write(b'%') # tells em to send extra line with pitch, roll etc
            line = connection.readline().decode('utf-8')
            if '$' in line:
                json_data = nmea_to_json(line)
                json_data['utc_datetime'] = datetime.datetime.utcnow().replace(microsecond=0).isoformat()
                full_line = json_data
                # try:
                status_dict = json.loads(self.mpstatus_to_json(self.mpstate.status))
                gps = status_dict['GPS_RAW_INT']
                lat = int(gps['lat']) / 1.0e7
                lon = int(gps['lon']) / 1.0e7
                full_line['lat'] = lat
                full_line['lon'] = lon
                full_line['sats_visible'] = int(gps['satellites_visible'])
                full_line['sat_fix'] = int(gps['fix_type'])
                if int(gps['fix_type']) > 1:
                    geom = f'POINT({str(lon)} {str(lat)})'
                    distance = 0
                    if self.em_last_point:
                        distance = geopy.distance.geodesic(self.em_last_point, (lat, lon))
                        print('DISTANCE', distance)
                        if distance > self.em_min_log_distance / 1000:
                            self.datapoints.append({'dataset': self.em_dataset_id, 'position': geom, 'data': full_line})
                            self.em_last_point = (lat, lon)
                    else:
                        self.em_last_point = (lat, lon)
                        self.datapoints.append({'dataset': self.em_dataset_id, 'position': geom, 'data': full_line})
                f = open(f'/home/pi/datastore/{self.em_file}.txt', 'a')
                f.write(json.dumps(full_line ) +'\n')
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

    connection.close()
    self.em_thread = False
