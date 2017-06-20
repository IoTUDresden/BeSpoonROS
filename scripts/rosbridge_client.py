#!/usr/bin/env python 

import websocket as ws 

class RosBridgeClient(Object):
    __turtlebot = "ws://192.168.1.56:9999"

    def __init__(self):
        self.conn = None                

    def connect(self, ws_url=None):
        if us_url is None: 
            ws_url = self.__turtlebot

        self.conn = ws.create_connection(ws_url)

        return self.conn.connected 

    def disconnect(self):
        if self.conn is not None:
            self.conn.shutdown()

    




