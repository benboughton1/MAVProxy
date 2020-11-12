import random
import json
import time

import geopy.distance


def rand_connect(self):
    while True:
        if self.allow_data_collection_threads:
            try:
                # generate random number and write to datastore and append to datapoints to send
                r = random.random()
                status_dict = json.loads(self.mpstatus_to_json(self.mpstate.status))
                gps = status_dict['GPS_RAW_INT']
                lat = int(gps['lat']) / 1.0e7
                lon = int(gps['lon']) / 1.0e7
                if int(gps['fix_type']) > 1:
                    geom = f'POINT({str(lon)} {str(lat)})'
                    distance = 0
                    if self.data_col_profiles['rand']['last_point']:
                        distance = geopy.distance.geodesic(self.data_col_profiles['rand']['last_point'], (lat, lon))
                        print('DISTANCE', distance)
                        if distance >= self.data_col_profiles['rand']['min_log_distance_m'] / 1000:
                            self.datapoints.append(
                                {'dataset': self.data_col_profiles['rand']['dataset_id'], 'position': geom,
                                 'data': {'value': r, 'lat': lat, 'lon': lon}})
                            self.data_col_profiles['rand']['last_point'] = (lat, lon)
                    else:
                        self.data_col_profiles['rand']['last_point'] = (lat, lon)
                        self.datapoints.append({'dataset': self.data_col_profiles['rand']['dataset_id'], 'position': geom,
                                                'data': {'value': r, 'lat': lat, 'lon': lon}})

            except Exception as e:
                print('Rand error', e)

            time.sleep(2)
        else:
            break
