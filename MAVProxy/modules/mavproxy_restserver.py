#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Server Module
Patrick Jose Pereira
April 2018
'''

import os
import time
import json
import socket
from threading import Thread

from flask import Flask, request

from flask import flash, redirect, url_for
from werkzeug.utils import secure_filename
UPLOAD_FOLDER = '.'
ALLOWED_EXTENSIONS = {'txt'}

from flask_cors import CORS
from werkzeug.serving import make_server
from MAVProxy.modules.lib import mp_module

import io
from contextlib import redirect_stdout


UPLOAD_FOLDER = '/path/to/the/uploads'
ALLOWED_EXTENSIONS = {'txt', 'pdf', 'png', 'jpg', 'jpeg', 'gif'}


def mavlink_to_json(msg):
    '''Translate mavlink python messages in json string'''
    ret = '\"%s\": {' % msg._type
    for fieldname in msg._fieldnames:
        data = getattr(msg, fieldname)
        ret += '\"%s\" : \"%s\", ' % (fieldname, data)
    ret = ret[0:-2] + '}'
    return ret


def mpstatus_to_json(status):
    '''Translate MPStatus in json string'''
    msg_keys = list(status.msgs.keys())
    data = '{'
    for key in msg_keys[:-1]:
        data += mavlink_to_json(status.msgs[key]) + ','
    data += mavlink_to_json(status.msgs[msg_keys[-1]])
    data += '}'
    return data


def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

class RestServer():
    '''Rest Server'''

    def __init__(self, state):
        # Set log level and remove flask output - Uncomment these lines to hide server log
        import logging
        self.log = logging.getLogger('werkzeug')
        self.log.setLevel(logging.ERROR)

        # Server variables
        self.app = None
        self.run_thread = None
        self.address = 'localhost'
        self.port = 5001

        # Save status
        self.status = None
        self.server = None

        self.f = io.StringIO()
        self.state = state

    def update_dict(self, mpstate):
        '''We don't have time to waste'''
        self.status = mpstate.status
        self.mpstate = mpstate

    def set_ip_port(self, ip, port):
        '''set ip and port'''
        self.address = ip
        self.port = port
        self.stop()
        self.start()

    def start(self):
        '''Stop server'''
        # Set flask
        self.app = Flask('RestServer')
        CORS(self.app)
        self.add_endpoint()
        # Create a thread to deal with flask
        self.run_thread = Thread(target=self.run)
        self.run_thread.start()

        self.app.secret_key = '123'
        self.app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER


    def running(self):
        '''If app is valid, thread and server are running'''
        return self.app != None

    def stop(self):
        '''Stop server'''
        self.app = None
        if self.run_thread:
            self.run_thread = None
        if self.server:
            self.server.shutdown()
            self.server = None

    def run(self):
        '''Start app'''
        with redirect_stdout(self.f):
            self.server = make_server(self.address, self.port, self.app, threaded=True)
            self.server.serve_forever()

    def request(self, arg=None):
        '''Deal with requests'''
        if not self.status:
            return '{"result": "No message"}'

        try:
            status_dict = json.loads(mpstatus_to_json(self.status))
        except Exception as e:
            print(e)
            return

        # If no key, send the entire json
        if not arg:
            return json.dumps(status_dict)

        # Get item from path
        new_dict = status_dict
        args = arg.split('/')
        for key in args:
            if key in new_dict:
                new_dict = new_dict[key]
            else:
                return '{"key": "%s", "last_dict": %s}' % (key, json.dumps(new_dict))

        return json.dumps(new_dict)

    def request_terminal(self, arg=None):
        response_lines = 10
        args = arg.split('/')
        if len(args) > 0:
            response_lines = int(args[0])
        return json.dumps({'response': self.f.getvalue().splitlines()[-response_lines:]})

    def request_cmd(self):
        data = request.get_json()
        self.mpstate.functions.process_stdin(data["cmd"])
        return json.dumps({"response": "command received"})

    def request_console(self, arg=None):
        try:
            status_dict = json.loads(mpstatus_to_json(self.status))
        except Exception as e:
            print(e)
            return
        console_values = self.mpstate.console.values.values
        console_text = self.mpstate.console.text.text
        response = {
            'response': {
            "values": console_values,
            "text": console_text,
            "mpstatus": {
                "armed": self.mpstate.status.armed,
                "HEARTBEAT": status_dict['HEARTBEAT'],
                "GLOBAL_POSITION_INT": status_dict['GLOBAL_POSITION_INT'],
                "GPS_RAW_INT": status_dict['GPS_RAW_INT'],
                "VFR_HUD": status_dict['VFR_HUD'],
                "MISSION_CURRENT": status_dict['MISSION_CURRENT']
                }
            }
        }
        return json.dumps(response)

    def request_waypoints(self):
        mission_list = self.mpstate.module('wp').wploader.view_list()

    def request_mission(self):
        if request.method == 'POST':
            data = request.get_json()
            f = open("mission.txt", "w")
            f.write('QGC WPL 110\n')
            try:
                for line in data["mission"]:
                    f.write(line.replace(' ', '\t') + '\n')
            except Exception as e:
                print(e)
                print(data)
            f.close()
            self.mpstate.functions.process_stdin('wp load mission.txt')
            return json.dumps({'response': 'mission accepted'})

    def add_endpoint(self):
        '''Set endpoits'''
        self.app.add_url_rule('/rest/mavlink/<path:arg>', 'rest', self.request)
        self.app.add_url_rule('/rest/mavlink/', 'rest', self.request)
        self.app.add_url_rule('/rest/terminal/<path:arg>', view_func=self.request_terminal, methods=['GET'])
        self.app.add_url_rule('/rest/cmd', view_func=self.request_cmd, methods=['POST'])
        self.app.add_url_rule('/rest/console', view_func=self.request_console, methods=['GET'])
        self.app.add_url_rule('/rest/mission', view_func=self.request_mission, methods=['POST'])



class ServerModule(mp_module.MPModule):
    ''' Server Module '''

    def __init__(self, mpstate):
        super(ServerModule, self).__init__(mpstate, "restserver", "restserver module")
        # Configure server
        self.rest_server = RestServer(self)
        m = mpstate

        # self.lat = None
        # self.lon = None
        # self.alt = None
        # self.speed = None
        # self.airspeed = None
        # self.groundspeed = None
        # self.heading = 0
        # self.wp_change_time = 0
        # self.fence_change_time = 0

        self.add_command('restserver', self.cmds, \
                         "restserver module", ['start', 'stop', 'address 127.0.0.1:4777'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: restserver <address|freq|stop|start>"

    def cmds(self, args):
        '''control behaviour of the module'''
        if not args or len(args) < 1:
            print(self.usage())
            return

        if args[0] == "start":
            if self.rest_server.running():
                print("Rest server already running.")
                return
            self.rest_server.start()
            print("Rest server running: %s:%s" % \
                  (self.rest_server.address, self.rest_server.port))

        elif args[0] == "stop":
            if not self.rest_server.running():
                print("Rest server is not running.")
                return
            self.rest_server.stop()

        elif args[0] == "address":
            # Check if have necessary amount of arguments
            if len(args) != 2:
                print("usage: restserver address <ip:port>")
                return

            address = args[1].split(':')
            # Check if argument is correct
            if len(address) == 2:
                self.rest_server.set_ip_port(address[0], int(address[1]))
                return

        else:
            print(self.usage())

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # Update server with last mpstate
        self.rest_server.update_dict(self.mpstate)

    def unload(self):
        '''Stop and kill everything before finishing'''
        self.rest_server.stop()
        pass

    def mavlink_packet(self, m):
        """handle an incoming mavlink packet"""

        # mtype = m.get_type()
        # if mtype == 'GPS_RAW':
        #    (self.lat, self.lon) = (m.lat, m.lon)
        # elif mtype == 'GPS_RAW_INT':
        #     (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
        # elif mtype == "VFR_HUD":
        #     self.heading = m.heading
        #     self.alt = m.alt
        #     self.airspeed = m.airspeed
        #     self.groundspeed = m.groundspeed

def init(mpstate):
    '''initialise module'''
    return ServerModule(mpstate)
