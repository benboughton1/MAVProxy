#!/usr/bin/env python
'''
Remote Module
Ben Boughton 2020

Parts of Server Module
Patrick Jose Pereira
April 2018
'''
import os
import os.path
import sys
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import textconsole
import json
import time
import datetime
import requests
import threading

from MAVProxy.modules.lib import mp_module

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from .data_random import rand_connect
from .data_em import em_connect

from .local_server import LocalServer
from .comm import comm


def mavlink_to_dict(msg):
    ret = {}
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret[fieldname] = data
    return ret


def mavlink_to_json(msg):
    """Translate mavlink python messages in json string
    - from restserver module"""
    ret = '\"%s\": {' % msg._type
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret += '\"%s\" : \"%s\", ' % (fieldname, data)
    ret = ret[0:-2] + '}'
    return ret


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
    """
    Capture vehicle alerts and report to API
    """

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
    def __init__(self, remote_self):
        self.text = []
        self.remote_self = remote_self

    def add(self, line):
        line["time"] = datetime.datetime.utcnow().isoformat()
        self.text.append(line)
        # trim to 100 lines
        # self.text = self.text[-100:]

        '''
        with self.remote_self.local_server.app.test_request_context('/'):
            self.remote_self.local_server.socketio.emit(
                'console',
                line,
                namespace='/ws'
            )        
        '''


class RemoteConsole(textconsole.SimpleConsole):
    def __init__(self, remote_self):
        super().__init__()
        self.stored_values = {}
        self.remote_self = remote_self
        self.stored_text = TextList(self.remote_self)
        self.server_alerts = ServerAlerts(self.remote_self)

    def write(self, text, fg='black', bg='white'):
        """write to the console"""

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


class CropqModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(CropqModule, self).__init__(mpstate, "cropq", "")

        self.mpstate.console = RemoteConsole(self)

        # configure local webserver
        self.config = None
        self.config_file = None
        self.local_server = LocalServer(self)
        self.local_server.start()

        # configure API communication
        self.comm_interval = 10  # seconds
        self.comm_start = False
        self.api_url = None
        self.username = None
        self.password = None
        self.vehicle_server_id = None

        # configure data collection
        self.allow_data_collection_threads = True
        self.data_col_profiles = {}

        self.datapoints = []

        self.last_comm = time.time()

        # self.status_callcount = 0
        self.packets_mytarget = 0
        self.packets_othertarget = 0

        self.status_text_sent = []
        self.io_text_sent = []
        self.params_last_sent = {}
        self.mpstats_last_sent = {}
        self.heartbeats_latest_20 = []
        self.comm_log_latest_100 = []

        self.direct_command_queue = []
        self.direct_commands = {}

        self.mav_ack_alerts = MavAckAlerts(self)

        self.remote_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ])

        self.add_command('cropq', self.cmd_cropq, "cropq module",
                         ['set (LOGSETTING)', 'i (seconds)', 'interval (seconds)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: example <status|set>"

    def load_config_file(self):
        print('loading config file')
        self.data_col_profiles = {}
        self.datapoints = []
        self.allow_data_collection_threads = False
        time.sleep(2)
        self.allow_data_collection_threads = True
        with open(self.config_file) as config_file:
            config = json.load(config_file)
            self.config = config
            self.api_url = config['local']['api']['url']
            self.username = config['local']['api']['username']
            self.password = config['local']['api']['password']
            self.vehicle_server_id = int(config['local']['api']['vehicle_id'])
            self.comm_interval = config['vehicle']['comm_interval_sec']
            # data collection setup
            self.data_collection_setup(config['vehicle']['data_collection'])
            if config['local']['auto_comm_start']:
                print(f'Starting comm in {str(config["local"]["comm_pause_sec"])} seconds')
                time.sleep(config['local']['comm_pause_sec'])
                self.comm_start = True
        print('config file loaded')

    def cmd_cropq(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "load_config":
            # try:
            self.config_file = args[1]
            self.load_config_file()
            # except Exception as e:
            #    print('error loading config file', e)
        elif args[0] == "reload_config":
            self.load_config_file()
        elif args[0] == "comm":
            comm()
            self.api_url = args[1]
            self.username = args[2]
            self.password = args[3]
            self.vehicle_server_id = int(args[4])
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

    def mpstatus_to_json(self, status):
        """Translate MPStatus in json string - from restserver module"""
        msg_keys = list(status.msgs.keys())
        data = '{'
        for key in msg_keys[:-1]:
            data += mavlink_to_json(status.msgs[key]) + ','
        data += mavlink_to_json(status.msgs[msg_keys[-1]])
        data += '}'
        return data

    def data_collection_setup(self, profile_list):
        for profile in profile_list:
            profile['file_name'] = None
            profile['thread'] = None
            profile['last_point'] = None
            profile['last_try'] = time.time()
            self.data_col_profiles[profile['profile_name']] = profile

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
        status_dict = json.loads(self.mpstatus_to_json(self.mpstate.status))
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
            x = threading.Thread(target=comm, args=(self,))
            x.start()

        # pop another job off the queue
        if len(self.direct_command_queue) > 0:
            self.direct_command_rover()

        # check to see if any active alerts have expired
        self.mpstate.console.server_alerts.check_timeout()

        # data collection
        profile_keys = list(self.data_col_profiles.keys())

        for profile_key in profile_keys:
            profile = self.data_col_profiles[profile_key]
            if now - profile['last_try'] > 3 and profile['on'] and not profile['thread']:
                print(f'Starting {profile["profile_name"]}')
                profile['thread'] = True
                profile['last_try'] = now
                if profile['profile_name'] == 'em':
                    y = threading.Thread(target=em_connect, args=(self,))
                    y.start()
                if profile['profile_name'] == 'rand':
                    z = threading.Thread(target=rand_connect, args=(self,))
                    z.start()


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
    return CropqModule(mpstate)
