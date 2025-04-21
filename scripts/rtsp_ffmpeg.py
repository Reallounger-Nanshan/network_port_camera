#!/home/uav/anaconda3/envs/SceneMatch/bin/python3.8
import ffmpeg
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

class RTSP():
    def __init__(self):
        # Initialize ROS
        rospy.init_node('rtsp_ffmpeg', anonymous = True)
        self.rtsp_url = rospy.get_param("~rtsp_url")
        self.is_rgb = rospy.get_param("~is_rgb", True)
        self.is_scale = rospy.get_param("~is_scale", True)
        new_size = rospy.get_param("~new_size", True) if self.is_scale else None
        self.new_imgsz = tuple(int(new_shape) for new_shape in new_size.split(" ")) if self.is_scale else None
        self.show_img = rospy.get_param("~show_img", False)
        self.topic_pub_name = rospy.get_param("~topic_pub_name")
        self.image_pub = rospy.Publisher(self.topic_pub_name, Image, queue_size = 1)
   
        # Get RTSP video stream information
        probe = ffmpeg.probe(self.rtsp_url)
        cap_info = next(x for x in probe['streams'] if x['codec_type'] == 'video')
        print("fps: {}".format(cap_info['r_frame_rate']))

        # Initialize ROS topic
        self.ros_img = Image()
        if self.is_scale:
            self.ros_img.width = self.new_imgsz[0]
            self.ros_img.height = self.new_imgsz[1]
            self.original_width = cap_info['width']
            self.original_height = cap_info['height']
        else:
            self.ros_img.width = self.original_width = cap_info['width']
            self.ros_img.height = self.original_height = cap_info['height']
        self.ros_img.encoding = "bgr8"

        # Create pipe
        args = {
            "rtsp_transport": "tcp",
            "fflags": "nobuffer",
            "flags": "low_delay"
        }
        self.process1 = (
            ffmpeg
            .input(self.rtsp_url, **args)
            .output('pipe:', format = 'rawvideo', pix_fmt = 'rgb24')
            .overwrite_output()
            .run_async(pipe_stdout = True)
        )


    def Run(self):
        while True:
            # Read image
            in_bytes = self.process1.stdout.read(self.original_width * self.original_height * 3)
            if not in_bytes:
                break
            
            # Convert ndarray
            in_frame = (
                np
                .frombuffer(in_bytes, np.uint8)
                .reshape([self.original_height, self.original_width, 3])
            )
        
            # Convert BGR
            if self.is_rgb:
                frame = cv2.cvtColor(in_frame, cv2.COLOR_RGB2BGR)
            else:
                frame_gray = in_frame[:, :, 0]
                frame = cv2.applyColorMap(frame_gray, cv2.COLORMAP_JET)

            # Scale image
            if self.is_scale:
                frame = cv2.resize(frame, self.new_imgsz, cv2.INTER_AREA)

            # Convert ROSImage
            self.ros_img.data = np.array(frame).tobytes()
            self.image_pub.publish(self.ros_img)

            if self.show_img:
                cv2.imshow("ffmpeg", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
            
        self.process1.kill()


if __name__ == "__main__":
    rtsp = RTSP()
    rtsp.Run()
    
    rospy.spin()
  
