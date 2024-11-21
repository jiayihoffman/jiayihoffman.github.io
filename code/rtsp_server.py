#!/usr/bin/env python3
import gi
from gi.repository import GLib

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer

class RTSPServer:
    def __init__(self):
        Gst.init(None)
        # Gst.debug_set_default_threshold(3)
        self.server = GstRtspServer.RTSPServer()
        self.factory = GstRtspServer.RTSPMediaFactory()
        self.factory.set_launch("libcamerasrc ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast ! rtph264pay name=pay0 pt=96")
        self.factory.set_shared(True)
        self.server.get_mount_points().add_factory("/stream", self.factory)
        self.server.attach(None)

    def run(self):
        print("RTSP server is running on rtsp://localhost:8554/stream")
        loop = GLib.MainLoop()
        loop.run()

if __name__ == "__main__":
    server = RTSPServer()
    server.run()

