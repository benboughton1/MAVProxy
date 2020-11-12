import json
import os
import datetime
import requests

def comm(self):
    '''
    Schedule POST to API
    Currently set it command line, TODO: manage by config file
    '''
    filename = '/tmp/rover-log.txt'

    if os.path.exists(filename):
        append_write = 'a'  # append if already exists
    else:
        append_write = 'w'  # make a new file if not

    with open(filename, append_write) as log:
        status_dict = json.loads(self.mpstatus_to_json(self.mpstate.status))
        try:
            lat = int(status_dict['GPS_RAW_INT']['lat']) / 1.0e7
            lon = int(status_dict['GPS_RAW_INT']['lon']) / 1.0e7
            position = f'POINT({str(lon)} {str(lat)})'
            fix_type = status_dict['GPS_RAW_INT']['fix_type']
            sats_visible = status_dict['GPS_RAW_INT']['satellites_visible']
            log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} INFO -- comm to write -- {lat} {lon} {sats_visible}\n')
        except Exception as e:
            lat = None
            lon = None
            position = None
            fix_type = None
            sats_visible = None
            log.write(f'ERROR 1 -- comm -- {e}\n')
            print(e)

        try:
            system_status = status_dict['HEARTBEAT']['system_status']
            custom_mode = status_dict['HEARTBEAT']['custom_mode']
        except Exception as e:
            log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} ERROR 2 -- comm -- {e}\n')
            system_status = None
            custom_mode = None
            print(e)

        try:
            heading = status_dict['VFR_HUD']['heading']
            speed_kmh = float(status_dict['VFR_HUD']['groundspeed']) * 3.6
        except Exception as e:
            heading = None
            speed_kmh = None
            log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} ERROR 3 -- comm -- {e}\n')
            print(e)

        text_to_save, text_to_send = self.console_text_to_send()
        params_to_send = self.params_to_send()
        mpstats_to_send = self.mpstats_to_send()

        datapoints_freeze = json.loads(json.dumps(self.datapoints))

        data = {
            "time_vehicle": datetime.datetime.utcnow().isoformat(),
            "armed": self.is_armed(),
            "position": position,
            "heading": heading,
            "speed": speed_kmh,
            "packets": 0,
            "fix_type": fix_type,
            "sats_visible": sats_visible,
            "system_status": system_status,
            "custom_mode": custom_mode,
            "console_texts": text_to_send,
            "parameters": params_to_send,
            "mpstats": mpstats_to_send,
            "datapoints": datapoints_freeze
        }

        data_small = {
            "time_vehicle": datetime.datetime.utcnow().isoformat(),
            "armed": self.is_armed(),
            "position": position,
            "heading": heading,
            "speed": speed_kmh,
            "fix_type": fix_type,
            "sats_visible": sats_visible,
            "datapoints": datapoints_freeze
        }

        self.heartbeats_latest_20 = [data_small] + self.heartbeats_latest_20
        self.heartbeats_latest_20 = self.heartbeats_latest_20[:19]
        # print(data)
        try:
            r = requests.post(f'{self.api_url}/vehicles/{self.vehicle_server_id}/heartbeats/',
                          auth=(self.username, self.password),
                          json=data
                          )

            if r.status_code == 201 or r.status_code == 200:
                # update sent mpstats & params
                self.status_text_sent += text_to_save
                for param in params_to_send:
                    self.params_last_sent[param['name']] = param['value']
                for mpstat in mpstats_to_send:
                    self.mpstats_last_sent[mpstat['name']] = mpstat['value']
                for dp in datapoints_freeze:
                    if dp in self.datapoints:
                        self.datapoints.remove(dp)

                # get new commands in response
                response = r.json()
                print(response)
                log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} INFO -- comm response -- {response}\n')
                self.read_direct_commands(response['direct_commands'])
            else:
                print(r.status_code, r.content)
                log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} INFO -- comm response -- bad response from server -- {r.status_code} {r.content}\n')

        except Exception as e:
            log.write(f'{datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")} ERROR 4 -- Caught making POST or dealing with response -- {e}\n')

    return
