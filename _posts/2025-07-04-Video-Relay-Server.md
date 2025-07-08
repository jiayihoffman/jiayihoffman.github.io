---
layout: post
title: "Live Video Streaming using a Media Server"
date: 2025-07-07 08:45:28 -0600
categories: Droid_Vision
---

In one of my early posts, I showed how to set up a streaming server on a mobile robot using GStreamer and visualize the robot’s surroundings from my mobile device. This works well if I’m using the robot to record videos of babies, pets, and flowers. Both the phone and the robot are on the same network, so the phone can directly access the robot’s IP address.

However, for a security camera use case, I am away, so my mobile device is outside the home network. The robot's IP address is not visible from the phone. How can I check on my home in real-time while I am away? Exposing the robot's IP address poses a significant security risk and is not an option.  

## Relay the Video using a Public Server
The solution is to use a public server to relay the video - The video stream from the robot is sent to a cloud server, which then re-publishes the stream to the mobile device. This way, the robot can stay within the private network with all the safety and security, while its video is accessible from a public media server protected by credentials and a firewall.

All video data sent to the server are in memory and pass through. Nothing is stored unless recording is requested. 

<a href="/assets/video_relay.drawio.png" target="_blank">
  <img src="/assets/video_relay.drawio.png" />
</a>

.

### MediaMTX RTMP Media Server 
A common relay option is using an RTMP server. RTMP is a protocol designed for streaming audio, video, and data over the internet, especially with low latency. It allows live video and audio from a source (such as a camera) to be sent to a platform (like YouTube Live or Twitch) for viewers to watch in real-time. 

The RTMP server I use is [MediaMTX](https://github.com/bluenviron/mediamtx). MediaMTX is open source and supports outputting live streams with RTSP, HLS, and WebRTC protocols. Therefore, the Droid Vision app can continue using RTSP, and I just need to change the RTSP URL to the public server, which is: `rtsp://PUBLIC_CLOUD_SERVER:8554/live/stream`.

MediaMTX is easy to run. I can run it as a Docker container or download and run its binaries. The binary download page is on [GitHub](https://github.com/bluenviron/mediamtx/releases). 

Here’s the output from MediaMTX. I used a Docker container. When the robot publishes the video to the MediaMTX server, we see something like “xxx is publishing to path ‘live/stream’”. When the Droid Vision app contacts the media server for the video stream, the message "xxx is reading from path ‘live/stream’” is displayed.
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
Here is the GStreamer pipeline to publish the Raspberry Pi's video stream to the RTMP server. Please replace PUBLIC_CLOUD_SERVER with your server's IP address in the cloud. 

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

 Here is the corresponding Python code "push_rtmp.py" for the robot:
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

I configure users and passwords in the mediamtx.yml config file, per path or for all paths.
```
# mediamtx.yml
server:
  # default bind ports
  rtspPort: 8554
  rtmpPort: 1935
  hlsPort: 8888
  webrtcPort: 8889

paths:
  all:
    # protect push
    publishUser: mypublisher
    publishPass: secret123

    # protect pull
    readUser: myviewer
    readPass: view456
```

On the robot, I change the rtmp_url of the RTMPPush class to:
```
self.rtmp_url = "rtmp://<username>:<password>@PUBLIC_CLOUD_SERVER/live/stream"
```

In Droid Vision app, I change the RTSP URL to:
```
rtsp://<username>:<password>@PUBLIC_CLOUD_SERVER:8554/live/stream
```

## On-Demand Video Streaming
With the public cloud server, another critical security monitoring enhancement is on-demand video streaming. Instead of streaming video continuously, the robot streams only when it detects an alert or upon the user's request. 

To be continued...