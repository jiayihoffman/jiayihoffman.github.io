---
layout: post
title: "Live Video Streaming of Security Cameras"
date: 2025-07-07 08:45:28 -0600
categories: Droid_Vision
image: /assets/media_server/IMG_3341.jpeg
---

[Droid Vision on an RC Truck]: {% link _posts/2024-11-18-DroidVision-RC.md %}

In one of my early posts [Droid Vision on an RC Truck], I showed how to set up a streaming server on a mobile robot using GStreamer and visualize the robot’s surroundings from my mobile device. This works well if I’m using the robot to record videos of pets, and flowers at home. Both the phone and the robot are on the same network, so the phone can directly access the robot’s IP address.

However, for a security camera use case, I am away, so my mobile device is outside the home network. The robot's IP address is not visible from the phone. How can I check on my home in real-time while I am away? Exposing the robot's IP address poses a significant security risk and is not an option.  

<img src="/assets/media_server/IMG_3341.jpeg" />

## Relay the Video using a Public Server
The solution is to use a public server to relay the video - The video stream from the robot is sent to a cloud server, which then re-publishes the stream to the mobile device. This way, the robot can stay within the private network with all the safety and security, while its video is accessible from a public media server protected by credentials and a firewall.

All video data sent to the server are in memory and pass through. Nothing is stored unless recording is requested. 

<a href="/assets/media_server/video_relay.drawio.png" target="_blank">
  <img src="/assets/media_server/video_relay.drawio.png" />
</a>

### MediaMTX RTMP Media Server 
A common relay option is using an RTMP server. RTMP is a protocol designed for streaming audio, video, and data over the internet, especially with low latency. It allows live video and audio from a source (such as a camera) to be sent to a platform (like YouTube Live or Twitch) for viewers to watch in real-time. 

The RTMP server I use is [MediaMTX](https://github.com/bluenviron/mediamtx), which is open source and supports outputting live streams with RTSP, HLS, and WebRTC protocols. Therefore, the Droid Vision app can continue using RTSP, and I just need to change the RTSP URL to the public server, which is: `rtsp://PUBLIC_CLOUD_SERVER:8554/live/stream`.

MediaMTX is easy to run. I can run it as a Docker container or download and run its binaries. The binary download page is on [GitHub](https://github.com/bluenviron/mediamtx/releases). 

Here’s the output from MediaMTX. I used the MediaMTX Docker container. When the robot publishes the video to the MediaMTX server, I see something like “xxx is publishing to path ‘live/stream’”. When the Droid Vision app contacts the media server for the video stream, the message "xxx is reading from path ‘live/stream’” is displayed.
```
% docker run --rm -it --network=host bluenviron/mediamtx:latest

025/07/07 21:02:36 INF MediaMTX v1.13.0
2025/07/07 21:02:36 INF configuration loaded from /mediamtx.yml
2025/07/07 21:02:36 INF [RTSP] listener opened on :8554 (TCP), :8000 (UDP/RTP), :8001 (UDP/RTCP)
2025/07/07 21:02:36 INF [RTMP] listener opened on :1935
2025/07/07 21:02:36 INF [HLS] listener opened on :8888
2025/07/07 21:02:36 INF [WebRTC] listener opened on :8889 (HTTP), :8189 (ICE/UDP)
2025/07/07 21:02:36 INF [SRT] listener opened on :8890 (UDP)
2025/07/07 21:03:01 INF [RTMP] [conn <robot ip>:34668] opened
2025/07/07 21:03:03 INF [RTMP] [conn <robot ip>:34668] is publishing to path 'live/stream', 1 track (H264)
2025/07/07 21:03:10 INF [RTSP] [conn <mobile device ip>:62634] opened
2025/07/07 21:03:10 INF [RTSP] [session c7b3d9d8] created by <mobile device ip>:62634
2025/07/07 21:03:11 INF [RTSP] [session c7b3d9d8] is reading from path 'live/stream', with UDP, 1 track (H264)

```

<!-- To enable recording through the MediaMTX media server add "runOnPublish: ffmpeg" option to the mediamtx.yml file.
```
paths:
  all:
    runOnPublish: ffmpeg -i rtsp://localhost:$RTSP_PORT/$RTSP_PATH -c copy myfile.mp4
``` -->

### Video Stream pushes to the RTMP Server
Here is the GStreamer pipeline for publishing the Raspberry Pi's video stream to the RTMP server. Please replace PUBLIC_CLOUD_SERVER with your server's IP address in the cloud.

```
gst-launch-1.0 libcamerasrc ! \
  video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
  videoconvert ! \
  queue ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 ! \
  queue ! \
  flvmux streamable=true name=mux ! \
  queue ! \
  rtmpsink location="rtmp://PUBLIC_CLOUD_SERVER/live/stream"
 ```

 Here is the corresponding Python code "push_rtmp.py" for the robot. Either method can publish the video stream to the RTMP server.
 ```
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class RTMPPush:
    def __init__(self):
        Gst.init(None)

        self.rtmp_url = "rtmp://PUBLIC_CLOUD_SERVER/live/stream"

        self.pipeline = Gst.parse_launch(
            f"""
            libcamerasrc !
            video/x-raw,format=YUY2,width=640,height=480,key-int-max=30,framerate=30/1 !
            videoconvert !
            queue !
            x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 !
            queue !
            flvmux streamable=true name=mux !
            queue !
            rtmpsink location={self.rtmp_url}
            """
        )
    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"Pushing stream to: {self.rtmp_url}")
        loop = GLib.MainLoop()
        loop.run()

if __name__ == "__main__":
    pusher = RTMPPush()
    pusher.run()

 ```

RTMP is very efficient, and MediaMTX is just a relay. Additionally, the receiving pipeline on Droid Vision uses UDP, which is extremely fast. So, there’s no noticeable delay when using the MediaMTX server to route the video stream.

### Video Stream fetched for Droid Vision
As mentioned earlier, we will continue to use RTSP to stream video to the mobile app. Therefore, the Droid Vision app only needs a URL update. The new URL is now: `rtsp://PUBLIC_CLOUD_SERVER:8554/live/stream`.

### Protect MediaMTX with Credentials 
By default, MediaMTX has no authentication — anyone who knows its RTSP/RTMP/HLS URL can push or pull. So it’s essential to protect it, especially when it runs on a public server. 

MediaMTX has built-in user/password auth for:
 * Publishers 
 * Readers/viewers

I configured users and passwords in the `mediamtx.yml` file this way. The credential applies to all paths. 

```
# mediamtx.yml
server:
  # default bind ports
  rtspPort: 8554
  rtmpPort: 1935
  hlsPort: 8888
  webrtcPort: 8889

paths:
  all_others:
    # protect push
    publishUser: mypublisher
    publishPass: secret123

    # protect pull
    readUser: myviewer
    readPass: view456
```

When publishing or reading the stream, I include the credentials in the URL. For example, on the robot, I change the rtmp_url of the RTMPPush class to:
```
self.rtmp_url = "rtmp://PUBLIC_CLOUD_SERVER/live/stream?user=mypublisher&pass=secret123"
```

In Droid Vision app, I change the RTSP URL to:
```
rtsp://myviewer:view456@PUBLIC_CLOUD_SERVER:8554/live/stream
```

This method is straightforward but not ideal for large-scale or highly secure applications due to hardcoded credentials. Therefore, MediaMTX has other methods using external authentication. For the detail, please check MediaMTX [product site](https://github.com/bluenviron/mediamtx).

## On-Demand Video Streaming
With the public cloud server, another critical enhancement is on-demand video streaming. Instead of streaming video continuously, the robot streams only when it detects an alert or upon the user's request. 

### Media Control Server
A quick solution is for the robot to run a small Python control client that periodically polls the cloud server for the streaming state, set by the mobile app. 

<a href="/assets/media_server/control_server.drawio.png" target="_blank">
  <img src="/assets/media_server/control_server.drawio.png" />
</a>

### MQTT Broker 
The second method involves using MQTT for push notifications: set up a public MQTT broker in the cloud. The robot subscribes to a specific MQTT topic, while the mobile app publishes `start` or `stop` messages to that topic. 

Compared to the previous solution, this method offers the advantage of eliminating polling, so the robot reacts instantly. It also scales well to many cameras. MQTT is widely used for smart cameras, doorbells, drones, and other devices, which is why I chose this approach for my security robot. 

I need to make some deployment adjustments to simplify the mobile app’s configuration: Instead of embedding the MQTT client library directly into the Droid Vision app, I used an MQTT bridge, which is an HTTP server running in the cloud that posts messages to the MQTT broker. This setup allows the Droid Vision app to continue using HTTP requests to control the streaming service on the robot. In a future update, I might add an MQTT Swift client library to Droid Vision so we can remove the MQTT bridge from the deployment.

<a href="/assets/media_server/MQTT.drawio.png" target="_blank">
  <img src="/assets/media_server/MQTT.drawio.png" />
</a>

#### 1. Install and Config Mosquitto MQTT Broker on the Cloud Server 

Update packages, install and config Mosquitto MQTT broker:

```
sudo apt update
sudo apt install mosquitto mosquitto-clients -y

# Enable and start Mosquitto service
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# add password auth
sudo mosquitto_passwd -c /etc/mosquitto/passwd mymqttuser
# Enter password when prompted

```

Edit Mosquitto config `/etc/mosquitto/conf.d/default.conf`:
```
allow_anonymous false
password_file /etc/mosquitto/passwd 
```

Restart Mosquitto for the config changes:
```
sudo systemctl restart mosquitto
```

#### 2. FastAPI MQTT Bridge on the Cloud Server

Install FastAPI and MQTT client:
```
pip install fastapi uvicorn paho-mqtt
```

Python code `bridge.py` for the MQTT bridge:
```
from fastapi import FastAPI
from paho.mqtt import publish

app = FastAPI()

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_USER = "mymqttuser"
MQTT_PASS = "..."
TOPIC = "camera/r4/stream"

@app.get("/")
def index():
    return {"msg": "MQTT Bridge running!"}

@app.post("/start")
def start():
    publish.single(
        TOPIC, "start",
        hostname=MQTT_BROKER,
        port=MQTT_PORT,
        auth={'username': MQTT_USER, 'password': MQTT_PASS}
    )
    return {"msg": "Published start"}

@app.post("/stop")
def stop():
    publish.single(
        TOPIC, "stop",
        hostname=MQTT_BROKER,
        port=MQTT_PORT,
        auth={'username': MQTT_USER, 'password': MQTT_PASS}
    )
    return {"msg": "Published stop"}
```
Start FastAPI MQTT Bridge:
```
uvicorn bridge:app --host 0.0.0.0 --port 8000
```

#### 3. Robot Streaming Control

The Raspberry Pi connects to the MQTT broker and waits for messages, then starts or stops the GStreamer accordingly.

```
import paho.mqtt.client as mqtt
import subprocess
import shlex

BROKER = "PUBLIC_CLOUD_SERVER"
PORT = 1883
USERNAME = "mymqttuser"
MQTT_PASS = "..."
TOPIC = "camera/r4/stream"

# the GStreamer pipeline created earlier
GST_PIPELINE = """
libcamerasrc ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! ...
"""

gst_process = None

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    global gst_process
    payload = msg.payload.decode()
    print(f"Received: {payload}")

    if payload == "start":
        if gst_process is None:
            print("Starting GStreamer...")
            gst_process = subprocess.Popen(
                shlex.split(f"gst-launch-1.0 {GST_PIPELINE}"),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
        else:
            print("Stream already running.")

    elif payload == "stop":
        if gst_process is not None:
            print("Stopping GStreamer...")
            gst_process.terminate()
            gst_process = None
        else:
            print("No stream to stop.")

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_forever()
```