from picamera2 import Picamera2, Preview
import time
from libcamera import Transform

fps = 100
picam2 = Picamera2()
resolution = (1640, 922)
camera_config = picam2.create_video_configuration(
    main={"size": resolution, "format": "BGR888"},
    lores={"size": resolution, "format": "YUV420"},
    display="main",
    controls={ # ALL camera controls for initial setup go here
        "FrameDurationLimits": [int(1000000/fps), int(1000000/fps)],
        "ExposureTime": 10000,
        "AeEnable": False    
    }
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)
picam2.capture_file("test.jpg")
picam2.stop()