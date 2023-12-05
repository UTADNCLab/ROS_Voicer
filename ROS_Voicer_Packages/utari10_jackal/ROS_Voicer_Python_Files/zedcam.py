import cv2
import pyzed.sl as sl
from yolodetect import Yolo_Obj
from robotmove import RobotMove
import threading
from voiceline import VoiceLine

class ZedCam:
    image_ocv = []

    def __init__(self):
        print("Initiating Camera...")
        # Creating ZED Camera Object
        self.zed = sl.Camera()
        # Open the camera
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:  # Ensure the camera has opened successfully
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()
        print("Camera Initiated.")
        self.stop_flag = threading.Event()
        recording_params = sl.RecordingParameters()
        recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H264
        recording_params.video_filename = 'VideoOutput.svo'
        err = self.zed.enable_recording(recording_params)

    def stop_cam_loop(self):
        print("Stopping Cam Loop...")
        self.stop_flag.set()
        print("Cam loop stopped.")

    def cam_loop(self):

        while not self.stop_flag.is_set():
            if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                image_ocv = image.get_data()
                ZedCam.image_ocv = image_ocv

                # Retrieve depth map. Depth is aligned on the left image
                self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                self.zed.retrieve_image(depth_view, sl.VIEW.DEPTH)

                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                depth_ocv = depth_view.get_data()
                show_camera_feed = True

                if show_camera_feed:
                    
                    # Display the disparity colormap, depth information, and input frame
                    #cv2.circle(image_ocv, (x, y), 5, (255, 255, 0), -1)
                    #cv2.imshow('Zed Camera Feed', image_ocv)
                    #cv2.imshow('Zed Depth Map', depth_ocv)

                    c = cv2.waitKey(1)

                    if c == 27:
                        print("Closing...")
                        cv2.destroyAllWindows()
                        print("All Objects Destroyed. Goodbye!")
                        break
                    elif c == 32:
                        ZedCam.depth_calc(image_ocv, x, y)
                    elif c == 13:
                        ZedCam.detector(image_ocv)
        else:
            ("Closing...")
            self.zed.disable_recording()
            self.zed.close()
            

    def depth_calc(image_ocv, x, y):
    	i = 0
    	while i <= 60:
	        err, pc_array = point_cloud.get_value(x, y)
	        pc_x = round(pc_array[0], 3)
	        pc_y = round(pc_array[1], 3)
	        pc_z = round(pc_array[2], 3)
	        print(f"Camera Frame Coordinates at: {{{x};{y}}}: ({pc_x},{pc_y},{pc_z})")
	        #cv2.imshow('Zed Camera Feed', image_ocv)
	        position = [-pc_x + 0.05, -pc_z -0.25, pc_y-0.05]
	        orientation = RobotMove.quarternion
	        print(f"Kinova Arm understood coordinates at: {position}")
	        
	        if Yolo_Obj.label:
		        print("Object is bottle!!!")
		        VoiceLine.found()
		        RobotMove.cartesian_pose_client(position, orientation)
		        Yolo_Obj.label = False
		        break
	        else:
		        print("Object is not bottle!!!")
		        pass
	        print(f"attempt no. : {i}")
	        i = i + 1
        
    def detector(image_ocv):
        Yolo_Obj.obj_detect(image_ocv)
        c_x = Yolo_Obj.c_x
        c_y = Yolo_Obj.c_y
        ZedCam.depth_calc(image_ocv, c_x, c_y)
        
# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
init_params.depth_minimum_distance = 0.3 # Set the minimum depth perception distance to 15cm

# Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.enable_fill_mode = False
runtime_parameters.confidence_threshold = 100
runtime_parameters.remove_saturated_areas = True
image = sl.Mat()
depth = sl.Mat()
depth_view = sl.Mat()
point_cloud = sl.Mat()
mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))

x = 640
y = 420
