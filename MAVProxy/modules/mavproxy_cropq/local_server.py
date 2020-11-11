import threading
import io
import pathlib

from flask import Flask, request, render_template, redirect
from flask_cors import CORS
from werkzeug.serving import make_server
from flask_socketio import SocketIO, Namespace, emit, send


class LocalServer():
    """
    Local server running so we can config via wifi connection
    """
    def __init__(self, state):
        # Set log level and remove flask output - Uncomment these lines to hide server log
        #import logging
        #self.log = logging.getLogger('socketio')
        #self.log.setLevel(logging.INFO)
        # self.log.setLevel(logging.ERROR)


        # Server variables
        self.app = None
        self.socketio = None
        self.run_thread = None
        self.address = '0.0.0.0'
        self.port = 5001

        self.websocket = None

        # Save status
        self.status = None
        self.server = None

        self.f = io.StringIO()
        self.state = state



    def set_ip_port(self, ip, port):
        '''set ip and port'''
        self.address = ip
        self.port = port
        self.stop()
        self.start()

    def start(self):
        '''Stop server'''
        # Set flask
        # run MAVProxy from one level
        current_directory = pathlib.Path(__file__).parent.absolute()
        print(current_directory)
        print(f'{current_directory}/templates')
        self.app = Flask('LocalServer',
                         template_folder=f'{current_directory}/templates',
                         static_folder=f'{current_directory}/static')
        # CORS(self.app)
        self.socketio = SocketIO(self.app, log_output=True, cors_allowed_origins='*', async_mode='threading')
        self.add_endpoint()
        # Create a thread to deal with flask
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()

        self.app.secret_key = '123'
        # self.app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER


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
        # with redirect_stdout(self.f):
        self.socketio.run(self.app, self.address, self.port, log_output=True)
        # self.server = make_server(self.address, self.port, self.socketio, threaded=True)
        # self.server.serve_forever()

    def request_console(self):
        return render_template('console.html')

    def request_vehicle(self):
        return render_template('vehicle.html')

    def request_dataset(self):
        return render_template('dataset.html')

    def request_index(self):
        redirect('/vehicle')

    def add_endpoint(self):
        self.app.add_url_rule('/console', view_func=self.request_console, methods=['GET'])
        self.app.add_url_rule('/vehicle', view_func=self.request_vehicle, methods=['GET'])
        self.app.add_url_rule('/dataset', view_func=self.request_dataset, methods=['GET'])
        self.app.add_url_rule('/', view_func=self.request_vehicle, methods=['GET'])

        self.websocket = Websocket(self, '/ws')
        self.socketio.on_namespace(self.websocket)


class Websocket(Namespace):
    def __init__(self, local_server_self, namespace=None):
        super(Namespace, self).__init__(namespace)
        self.socketio = None
        self.local_server_self = local_server_self

    def on_connect(self):
        print('LOCAL SERVER WEB SOCKET CONNECTED')
        pass

    def on_disconnect(self):
        print('LOCAL SERVER WEB SOCKET DISCONNECTED')
        pass

    def on_my_event(self, data):
        print('My EVENT', data)
        emit('my_response', data)

    def emit_console(self, data):
        emit('console', data)
