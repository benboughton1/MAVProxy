import threading
import io

from flask import Flask, request
from flask_cors import CORS
from werkzeug.serving import make_server


class LocalServer():
    """
    Local server running so we can config via wifi connection
    """
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

    def set_ip_port(self, ip, port):
        '''set ip and port'''
        self.address = ip
        self.port = port
        self.stop()
        self.start()

    def start(self):
        '''Stop server'''
        # Set flask
        self.app = Flask('LocalServer')
        CORS(self.app)
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
        self.server = make_server(self.address, self.port, self.app, threaded=True)
        self.server.serve_forever()

    def request_test(self, arg=None):
        return 'hello'

    def add_endpoint(self):
        self.app.add_url_rule('/', view_func=self.request_test, methods=['GET'])
