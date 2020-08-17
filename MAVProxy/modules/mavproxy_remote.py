#!/usr/bin/env python
'''
Remote Module
Ben Boughton 2020

Description here
'''

import os
import os.path
import sys
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import textconsole
import errno
import json
import time
import datetime
import requests
import serial
import threading
import random
import geopy.distance
import logging
import logging.config

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

def nmea_to_json(line):
    split = line.split('*')
    data = split[0].split(',')
    if data[0] == '$PDLM1':
        return {
            'array_length': 1,
            'time': data[1],
            'hcp_conductivity': float(data[2]),
            'hcp_inphase': float(data[3]),
            'prp_conductivity': float(data[4]),
            'prp_inphase': float(data[5])
        }

    if data[0] == '$PDLMA':
        return {
            'voltage': float(data[1]),
            'temperature': float(data[2]),
            'pitch': float(data[3]),
            'roll': float(data[4])
        }


def mavlink_to_dict(msg):
    ret = {}
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret[fieldname] = data
    return ret


def mavlink_to_json(msg):
    '''Translate mavlink python messages in json string
    - from restserver module'''
    ret = '\"%s\": {' % msg._type
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret += '\"%s\" : \"%s\", ' % (fieldname, data)
    ret = ret[0:-2] + '}'
    return ret


def mpstatus_to_json(status):
    '''Translate MPStatus in json string - from restserver module'''
    msg_keys = list(status.msgs.keys())
    data = '{'
    for key in msg_keys[:-1]:
        data += mavlink_to_json(status.msgs[key]) + ','
    data += mavlink_to_json(status.msgs[msg_keys[-1]])
    data += '}'
    return data


class MavAckAlerts():
    def __init__(self, remote_self):
        self.active = []
        self.remote_self = remote_self

    def add(self, trigger, timeout):
        finish_time = time.time() + timeout
        self.active.append({
            'finish_time': finish_time,
            'trigger': trigger
        })

    def check_alerts(self, ack):
        now = time.time()
        for alert in list(self.active):
            trigger = alert['trigger']
            if trigger['mav_type'] == ack['type'] and trigger['mav_cmd_name'] == ack['name']:
                # print(f'Mav Alerts - match, {trigger["mav_type"]}')
                self.active.remove(alert)
                self.remote_self.update_direct_command(
                    trigger['direct_command_pk'],
                    None,
                    ack['result']
                )
                continue
            if now > alert['finish_time']:
                # print(f'Mav Alerts - timeout, {trigger["mav_type"]}')
                self.active.remove(alert)


class ServerAlerts():
    def __init__(self, remote_self):
        self.active = []
        self.remote_self = remote_self

    def add(self, trigger, timeout):
        finish_time = time.time() + timeout
        self.active.append({
            'finish_time': finish_time,
            'trigger': trigger
        })

    def check_alerts(self, console_line):
        now = time.time()
        for alert in list(self.active):
            trigger = alert['trigger']
            if trigger['console_text'] in console_line:
                self.active.remove(alert)
                self.remote_self.update_direct_command(
                    trigger['direct_command_pk'],
                    trigger['report_status']
                )
                continue
            if now > alert['finish_time']:
                self.active.remove(alert)

    def check_timeout(self):
        now = time.time()
        for alert in list(self.active):
            if now > alert['finish_time']:
                self.active.remove(alert)


class TextList():
    def __init__(self):
        self.text = []

    def add(self, line):
        line["time"] = datetime.datetime.utcnow().isoformat()
        self.text.append(line)
        # trim to 100 lines
        # self.text = self.text[-100:]


class RemoteConsole(textconsole.SimpleConsole):
    def __init__(self, remote_self):
        super().__init__()
        self.stored_text = TextList()
        self.stored_values = {}
        self.remote_self = remote_self
        self.server_alerts = ServerAlerts(self.remote_self)

    def write(self, text, fg='black', bg='white'):
        '''write to the console'''

        # wrap in try in case bad requests from server
        try:
            self.server_alerts.check_alerts(text)
        except Exception as e:
            print(e)

        if isinstance(text, str):
            sys.stdout.write(text)
        else:
            sys.stdout.write(str(text))
        sys.stdout.flush()
        self.stored_text.add({"text": text, "fg": fg, "bg": bg})

    def set_status(self, name, text='', row=0, fg='black', bg='white'):
        '''set a status value'''
        self.stored_values[name] = {"text": text, "fg": fg, "bg": bg, "row": row}


class Remote(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(Remote, self).__init__(mpstate, "remote", "")

        self.mpstate.console = RemoteConsole(self)

        self.comm_interval = 10  # seconds
        self.comm_start = False
        self.api_url = None
        self.username = None
        self.password = None
        self.vehicle_server_id = None

        self.em_enabled = False
        self.em_file = None
        self.em_thread = False
        self.em_dataset_id = None
        self.em_last_point = None
        self.em_min_log_distance = None
        self.last_em_try = time.time()

        self.rand_enabled = False
        self.rand_file = None
        self.rand_thread = False
        self.rand_dataset_id = None
        self.rand_last_point = None
        self.rand_min_log_distance = None
        self.last_rand_try = time.time()

        self.datapoints = [];

        self.last_comm = time.time()

        # self.status_callcount = 0
        self.packets_mytarget = 0
        self.packets_othertarget = 0

        self.status_text_sent = []
        self.io_text_sent = []
        self.params_last_sent = {}
        self.mpstats_last_sent = {}

        self.direct_command_queue = []
        self.direct_commands = {}

        self.mav_ack_alerts = MavAckAlerts(self)

        self.em_connection = None

        self.remote_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ])

        self.add_command('remote', self.cmd_remote, "remote module",
                         ['set (LOGSETTING)', 'i (seconds)', 'interval (seconds)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: example <status|set>"

    def cmd_remote(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "comm":
            self.comm()
            self.api_url = args[1]
            self.username = args[2]
            self.password = args[3]
            self.vehicle_server_id = int(args[4])
        elif args[0] == "comm_start":
            # TODO: type check all arguments
            self.api_url = args[1]
            self.username = args[2]
            self.password = args[3]
            self.vehicle_server_id = int(args[4])
            self.comm_interval = int(args[5])
            try:
                on_at_start = args[7].split(',')
                for x in on_at_start:
                    details = x.split('|')
                    print(details)
                    if details[0] == 'em':
                        print('Enabling EM')
                        self.em_enabled = True
                        self.em_dataset_id = int(details[1])
                        try:
                            self.em_min_log_distance = int(details[2])
                        except:
                            print('No EM min log distance set - setting to 7')
                            self.em_min_log_distance = 7
                    if details[0] == 'rand':
                        print('Enabling Random Number Generator')
                        self.rand_enabled = True
                        self.rand_dataset_id = int(details[1])
                        try:
                            self.rand_min_log_distance = int(details[2])
                        except:
                            print('No Rand min log distance set - setting to 7')
                            self.rand_min_log_distance = 7
            except Exception as e:
                print(e)
                pass
            try:
                time.sleep(args[6])
                self.comm_start = True
            except Exception as e:
                self.comm_start = True
        elif args[0] == "comm_stop":
            self.comm_start = False
        elif args[0] == "set_comm_interval":
            self.comm_interval = int(args[1])
        elif args[0] == "dl_mission":
            self.download_mission_file(args[1], args[2])
        elif args[0] == "set":
            self.remote_settings.command(args[1:])
        else:
            print(self.usage())

    def download_mission_file(self, job, mission):
        url = f'{self.api_url}/jobs/{job}/missions/{mission}/text_file/'
        response = requests.get(url, auth=(self.username, self.password))
        if response.status_code == 200:
            with open('mission.txt', 'wb') as f:
                f.write(response.content)
            self.console.writeln('Mission download successful')
        else:
            self.console.writeln(f'Mission download fail. Server status: {response.status_code}')

    def is_armed(self):
        if self.mpstate.status.armed == 128:
            return True
        else:
            return False

    def console_text_to_send(self):
        text_to_send = []
        text_to_save = []
        for text in self.mpstate.console.stored_text.text:
            if text not in self.status_text_sent:
                t = text['text']
                if text['text'] == ' ':
                    t = '_'
                if text['text'] == '\n':
                    t = '_\n'

                console_text = {
                    'time': text['time'],
                    'text': t,
                    'color_bg': text['bg'],
                    'color_fg': text['fg']
                }
                text_to_save.append(text)
                text_to_send.append(console_text)
        return text_to_save, text_to_send

    def mpstats_to_send(self):
        status_dict = json.loads(mpstatus_to_json(self.mpstate.status))
        mpstats_to_send = []

        for key, value in status_dict.items():
            for key_, value_ in value.items():
                key__ = f'{key}__{key_}'
                try:
                    existing = self.mpstats_last_sent[key__]
                except Exception as e:
                    existing = None

                if existing != value_:
                    mpstats_to_send.append({
                        'name': key__,
                        'value': value_
                    })

        return mpstats_to_send

    def params_to_send(self):
        param = self.mpstate.mav_param_by_sysid[(1, 1)]
        params_to_send = []

        for key, value in param.items():
            try:
                existing = self.params_last_sent[key]
            except Exception as e:
                existing = None

            if existing != value:
                params_to_send.append({
                    'name': key,
                    'value': value
                })

        return params_to_send

    def update_direct_command(self, direct_command_pk, status=None, mav_status=None):
        '''
        Sets status of command on server
        status:
            0. command waiting to be sent
            1. command vehicle received and added to queue
            2. command executed - success match
            3. command executed - fail match
            4. command executed - timeout without match
        '''
        data = {}
        if mav_status is not None:
            data['mav_status'] = mav_status
        if status is not None:
            data['status'] = status
        if status == 1:
            data["time_vehicle_start"] = datetime.datetime.utcnow().isoformat()
        if status in [2, 3, 4]:
            data["time_vehicle_finish"] = datetime.datetime.utcnow().isoformat()

        url = f'{self.api_url}/vehicles/{self.vehicle_server_id}/mavproxy_commands/{direct_command_pk}/'
        r = requests.patch(url, auth=(self.username, self.password), json=data)
        print(r.text)

    def read_direct_commands(self, direct_commands):
        '''
        Reads commands send from server,
        updates command dict puts them in queue
        '''
        for direct_command in direct_commands:
            self.direct_commands[direct_command['id']] = direct_command
            if direct_command['status'] == 0:
                self.update_direct_command(direct_command['id'], 1)
                self.direct_command_queue.append(direct_command['id'])

    def check_em_connection(self):
        pass

    def rand_connect(self):
        while True:
            try:
                # generate random number and write to datastore and append to datapoints to send
                r = random.random()
                status_dict = json.loads(mpstatus_to_json(self.mpstate.status))
                gps = status_dict['GPS_RAW_INT']
                lat = int(gps['lat']) / 1.0e7
                lon = int(gps['lon']) / 1.0e7
                if int(gps['fix_type']) > 1:
                    geom = f'POINT({str(lon)} {str(lat)})'
                    distance = 0
                    if self.rand_last_point:
                        distance = geopy.distance.geodesic(self.rand_last_point, (lat, lon))
                        print('DISTANCE', distance)
                        if distance > self.rand_min_log_distance / 1000:
                            self.datapoints.append({'dataset': self.rand_dataset_id, 'position': geom,
                                                    'data': {'value': r, 'lat': lat, 'lon': lon}})
                            self.rand_last_point = (lat, lon)
                    else:
                        self.rand_last_point = (lat, lon)
                        self.datapoints.append({'dataset': self.rand_dataset_id, 'position': geom,
                                                'data': {'value': r, 'lat': lat, 'lon': lon}})

            except Exception as e:
                print('Rand error', e)

            time.sleep(2)


    def em_connect(self):
        connection = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)
        connection.write(b'%')
        self.em_file = f'em_{datetime.datetime.utcnow().replace(microsecond=0).isoformat()}'

        count = 0

        while True:
            try:
                line1 = connection.readline().decode('utf-8')
                line2 = connection.readline().decode('utf-8')
                if '$' not in line1:
                    print('$ not line line 1')
                    connection.write(b'%')
                    time.sleep(1)
                    pass
                else:
                    json_data1 = nmea_to_json(line1)
                    json_data2 = nmea_to_json(line2)
                    part_line = {**json_data1, **json_data2}
                    part_line['utc_datetime'] = datetime.datetime.utcnow().replace(microsecond=0).isoformat()
                    full_line = part_line
                    #try:
                    status_dict = json.loads(mpstatus_to_json(self.mpstate.status))
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
                    f.write(json.dumps(full_line)+'\n')
                    f.close()
                    count = 0
                    #time.sleep(1)
            except Exception as e:
                print('Problem accessing EM serial - this may resolve itself', count)
                print(e)
                count = count + 1
                if count == 10:
                    print('10 error in row - reconnect')
                    break
            time.sleep(0.2)

        connection.close()
        self.em_thread = False 

    def comm(self):
        filename = '/tmp/rover-log.txt'

        if os.path.exists(filename):
            append_write = 'a'  # append if already exists
        else:
            append_write = 'w'  # make a new file if not

        with open(filename, append_write) as log:
            status_dict = json.loads(mpstatus_to_json(self.mpstate.status))
            try:
                lat = int(status_dict['GPS_RAW_INT']['lat']) / 1.0e7
                lon = int(status_dict['GPS_RAW_INT']['lon']) / 1.0e7
                position = f'POINT({str(lon)} {str(lat)})'
                fix_type = status_dict['GPS_RAW_INT']['fix_type']
                sats_visible = status_dict['GPS_RAW_INT']['satellites_visible']
                log.write(f'INFO -- comm to write -- {lat} {lon} {sats_visible}\n')
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
                log.write(f'ERROR 2 -- comm -- {e}\n')
                system_status = None
                custom_mode = None
                print(e)

            try:
                heading = status_dict['VFR_HUD']['heading']
                speed_kmh = float(status_dict['VFR_HUD']['groundspeed']) * 3.6
            except Exception as e:
                heading = None
                speed_kmh = None
                log.write(f'ERROR 1 -- comm -- {e}\n')
                print(e)

            text_to_save, text_to_send = self.console_text_to_send()
            params_to_send = self.params_to_send()
            mpstats_to_send = self.mpstats_to_send()

            datapoints_freeze = self.datapoints

            print('datapoints_freeze', datapoints_freeze)

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

            # print(data)

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
                log.write(f'INFO -- comm response -- {response}\n')
                self.read_direct_commands(response['direct_commands'])
            else:
                print(r.status_code, r.content)
                log.write(f'INFO -- comm response -- bad response from server -- {r.status_code} {r.content}\n')



    def direct_command_rover(self):
        direct_command_pk = self.direct_command_queue.pop()
        direct_command = self.direct_commands[direct_command_pk]
        mav_cmd_def = direct_command['mav_cmd_def']
        # Execute command  ----
        # self.mpstate.module('arm').
        cmd = ''
        for arg in mav_cmd_def["args_fixed"]:
            cmd = cmd + f' {arg}'
        for arg in direct_command["args"]:
            cmd = cmd + f' {arg}'
        print('----->', cmd)
        self.mpstate.functions.process_stdin(cmd)
        if mav_cmd_def['track_console']:
            for line in mav_cmd_def['console_text_success']:
                self.console.server_alerts.add({
                    'direct_command_pk': direct_command_pk,
                    'console_text': line,
                    'report_status': 2,
                }, mav_cmd_def['timeout'])
            for line in mav_cmd_def['console_text_fail']:
                self.console.server_alerts.add({
                    'direct_command_pk': direct_command_pk,
                    'console_text': line,
                    'report_status': 3,
                }, mav_cmd_def['timeout'])
        if mav_cmd_def['track_mav_msg']:
            self.mav_ack_alerts.add({
                'direct_command_pk': direct_command_pk,
                'mav_type': mav_cmd_def['mav_type'],
                'mav_cmd_name': mav_cmd_def['mav_cmd_name']
            }, mav_cmd_def['timeout'])

    def idle_task(self):
        '''called rapidly by mavproxy'''

        now = time.time()

        # send comm at set interval
        if now - self.last_comm > self.comm_interval and self.comm_start:
            self.last_comm = now
            self.comm()

        # pop another job off the queue
        if len(self.direct_command_queue) > 0:
            self.direct_command_rover()

        # check to see if any active alerts have expired
        self.mpstate.console.server_alerts.check_timeout()

        if now - self.last_em_try > 3 and self.em_enabled and not self.em_thread:
            print('starting em serial')
            self.em_thread = True;
            self.last_em_try = now
            x = threading.Thread(target=self.em_connect)
            x.start()

        if now - self.last_rand_try > 3 and self.rand_enabled and not self.rand_thread:
            print('start rand')
            self.rand_thread = True
            self.last_rand_try = now
            x = threading.Thread(target=self.rand_connect)
            x.start()

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        # check system Id - Param class uses this
        # sysid = (m.get_srcSystem(),m.get_srcComponent())
        # print(sysid)

        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

        mtype = m.get_type()

        if mtype == "COMMAND_ACK" or mtype == "MISSION_ACK":
            try:
                cmd = mavutil.mavlink.enums["MAV_CMD"][m.command].name
                res = mavutil.mavlink.enums["MAV_RESULT"][m.result].name
                msg = "Got COMMAND_ACK: %s: %s" % (cmd, res)
                self.mav_ack_alerts.check_alerts({
                    'type': mtype,
                    'name': cmd,
                    'result': res
                })
                print(msg)
            except Exception:
                msg = "Got MAVLink msg: %s" % m
                print(msg)


def init(mpstate):
    '''initialise module'''
    return Remote(mpstate)
