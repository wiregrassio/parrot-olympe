import math, os, logging, time
from multiprocessing.sharedctypes import Value

#loading olymoe takes 10-15 seconds on Nano
print ('\nStarting...\n')
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, Emergency, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages import gimbal, camera

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(name)s - %(message)s')
olympe_log = logging.getLogger('olympe').setLevel(logging.WARNING)
log = logging.getLogger(__name__)

DRONE_IP = os.environ.get('DRONE_IP', '192.168.42.1') #default IP if connecting directly to physical drone

class Parrot_Drone:
    def __init__(self, IP_address):
        self.drone = olympe.Drone(IP_address)
        self.drone.connect()

        assert self.drone(gimbal.set_max_speed(
            gimbal_id = 0, 
            yaw = 0,
            pitch = 180,
            roll = 90, 
            _timeout = 5
        )).wait().success()

        return None

    def __del__(self):
        self.drone.disconnect()
        return True

    def get_GPS_fix(self, retries = 1, timeout = 60):
        for x in range(retries):
            log.info('Attempt %d to get GPS Lock...' % (x + 1))
            resp = self.drone(GPSFixStateChanged(fixed=1, _timeout=timeout)).wait()

            if resp.success():
                log.info('Aquired GPS Lock')
                return True

            elif resp.timedout():
                log.info('Trying Again')

        log.error('Failed to get GPS Lock')
        return False

    def get_position(self):
        try:
            resp = self.drone.get_state(PositionChanged)
            pos = {'lat': resp['latitude'], 'long': resp['longitude'], 'AGL': resp['altitude']}

            if pos['lat'] > 90 or pos['lat'] < -90:
                raise ValueError

            elif pos['long'] > 180 or pos['long'] < -180:
                raise ValueError

            elif pos['AGL'] > 200 or pos['AGL'] < -200:
                raise ValueError

        except RuntimeError:
            log.error("Can't Get Position Without Fix")
            return False

        except ValueError:
            log.error('Coordinates out of Bounds')
            return False
        
        else:
            return pos

# in case of critical failure, kill motors. THIS WILL CAUSE THE DRONE TO FALL IF IN THE AIR!!!!!!!
    def emergency(self):
        log.critical('Cutting Motors')
        assert self.drone(Emergency()).wait().success()

# takeoff waits until state change to hovering
    def takeoff(self, timeout = 5):
        if (self.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.landed):
            log.info('Taking Off')
            assert self.drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=timeout)).wait().success()
            return True

        else:
            log.warning('Already Flying')
            return False

# will override any current command and land immediately
    def land(self, timeout = 30):
        if (self.drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.landed):
            resp = self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=timeout)).wait()
            if resp.success():
                return True
            
            else:
                self.emergency() #if for some reason landing hits timeout, assume critical failure and kill motors
                return False

        else:
            log.warning('Already Landed')
            return False

# moves relative to current orientation. distances are in meters, angles are in degrees
    def move_rel(self, forward=0, backward=0, right=0, left=0, up=0, down=0, turn_right=0, turn_left=0, timeout = 30):
        turn_angle = (turn_right - turn_left) * math.pi / 180
        if (self.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.hovering):
            resp = self.drone(moveBy(forward-backward, right-left, down-up, turn_angle) >> FlyingStateChanged(state="hovering", _timeout=timeout)).wait()
            if resp.success():
                return True

            elif resp.timedout():
                # Add cancel moveby if move times out
                log.error('Move Timed Out')
                return False

        else:
            log.warning('Must be Hovering to Initiate Move')
            return False

# moves gimbal to specified position in degrees
    def gimbal_pitch_abs(self, angle_deg, timeout = 5):
        angle_deg = int(-90 if angle_deg < -90 else 90 if angle_deg > 90 else angle_deg)

        log.info('Moving Gimbal to %d Degrees' % angle_deg)
        resp = self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="none",
            yaw=0.0,
            pitch_frame_of_reference="absolute",
            pitch=angle_deg,
            roll_frame_of_reference="none",
            roll=0.0,
            _timeout=5
        ) >>
        gimbal.attitude(pitch_absolute=angle_deg, _timeout=timeout)).wait()

        if resp.success():
            return True

        if resp.timedout():
            log.warning('Gimbal Move Timed Out')
            return False

        elif not resp.success():
            log.error('Unknown Gimbal Error')
            # Add recalibrate gimbal if unknown error occurs 
            return False




    def setup_camera(self, mode = 'photo'):
        if mode == 'photo':
            self.drone(camera.set_camera_mode(cam_id=0, value='photo')).wait() 

            self.drone(camera.set_photo_mode(
                cam_id = 0,
                mode = 'single',
                format = 'full_frame',
                file_format = 'jpeg',
                burst = 'burst_4_over_1s',
                bracketing = 'preset_1ev',
                capture_interval = 0
            )).wait()

        elif mode == 'video':
            self.drone(camera.set_camera_mode(cam_id = 0, value = 'recording')).wait()
#            self.drone(camera.set_recording_mode())

#        elif mode == 'stream':
#            self.drone(camera.set_streaming_mode())



if __name__ == "__main__":

#---------- Init ----------#
    log.info('Creating Drone...')
    my_drone = Parrot_Drone(DRONE_IP)

    try:
        assert my_drone.get_GPS_fix(timeout=60, retries = 2)
        time.sleep(1)
        pos = my_drone.get_position()
        log.info(pos)
        input('---------- Press Enter to Continue ----------')

#---------- End Init ----------#


#---------- User Instructions ----------#
#        my_drone.setup_camera(mode = 'photo')

        my_drone.takeoff()

        my_drone.move_rel(backward = 20, up =2)
        my_drone.move_rel(turn_left = 30)
        my_drone.move_rel(forward = 40)
        my_drone.move_rel(turn_right = 120)
        my_drone.move_rel(forward = 40)
        my_drone.move_rel(turn_right = 120)
        my_drone.move_rel(forward = 40)
        my_drone.move_rel(turn_right = 150)
        my_drone.move_rel(forward = 20)
#        my_drone.move_rel(down = 1)

#---------- End User Instructions ----------#


#---------- Cleanup ----------#
    finally:
        my_drone.land() #land if not already landed
        my_drone.__del__() 

#---------- End Cleanup ----------#