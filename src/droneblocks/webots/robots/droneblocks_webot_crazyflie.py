import random
import time
from dataclasses import dataclass
from enum import Enum
from math import cos, sin
from typing import Any

from controller import Keyboard
from controller import Robot

from droneblocks.crazyflie.pid_controller import PIDController


@dataclass(frozen=False)
class PreFlyReturnData:
    short_circuit: bool = False  # True - then return from Fly function without executing rest
    # False - update the Fly function params with the values in this dataclass and continue
    forward_desired: float = 0.0
    sideways_desired: float = 0.0
    yaw_desired: int = 0
    height_diff_desired: float = 0.0


class DroneBlocksWebotCrazyflieRobot(Robot):

    def __init__(self):
        super().__init__()
        self.range_right = None
        self.range_back = None
        self.range_left = None
        self.range_front = None
        self.camera = None
        self.gyro = None
        self.gps = None
        self.imu = None
        self.pid_controller = PIDController()
        self.m1_motor = None
        self.m2_motor = None
        self.m3_motor = None
        self.m4_motor = None
        self.first_step: bool = False
        self.past_x_global = 0
        self.past_y_global = 0
        self.height_desired = 0.2
        self.past_time = 0
        self.autonomous_mode: bool = False
        self.emitter = None
        self.receiver = None

        # The queen bee is the one bee that the other worker bees follow when
        # they swarm.  There can only be a single queen been in a swarm
        # If this drone is the queen, then it will send ( emit ) flying commands
        # for the other 'worker' bees to follow
        self.is_queen_bee: bool = False

        # worker bees listen for commands from the queen bee
        self.is_worker_bee: bool = False

        self._init_motors()

    def _init_motors(self):
        # Initialize motors
        self.m1_motor = self.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

    def init_sensors(self, timestep):
        # Initialize Sensors
        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(timestep)
        self.gps = self.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(timestep)
        self.camera = self.getDevice("camera")
        self.camera.enable(timestep)
        self.range_front = self.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.getDevice("range_right")
        self.range_right.enable(timestep)
        self.emitter = self.getDevice("crazyflie_emitter")
        if self.emitter is None:
            print("******************* NO EMITTER FOUND")
        self.receiver = self.getDevice("crazyflie_receiver")
        if self.receiver is None:
            print("******************* NO RECEIVER FOUND")
        else:
            self.receiver.enable(timestep)

    def initialize(self) -> int:
        timestep = int(self.getBasicTimeStep())

        self.init_sensors(timestep)
        self.init_keyboard(timestep)

        return timestep

    def set_queen_bee(self, flag: bool) -> None:
        if self.is_worker_bee:
            raise Exception("Drone is already a worker bee.  You have to unset the worker bee status first")
        self.is_queen_bee = flag

    def set_worker_bee(self, flag: bool) -> None:
        if self.is_queen_bee:
            raise Exception("Drone is already a queen bee.  You have to unset the queen bee status first")
        self.is_worker_bee = flag

    def is_autonomous_flight(self) -> bool:
        return bool(self.autonomous_mode)

    def pre_fly(self, forward_desired: float = 0.0,
                sideways_desired: float = 0.0,
                yaw_desired: int = 0,
                height_diff_desired: float = 0.0) -> PreFlyReturnData:
        """
        Method is meant to be overridden by subclasses to process flying BEFORE the standard implementation.
        If this method is not overridden, it will be called and always return True.
        True - continue processing the standard flying behavior.
        False - return from the fly method, and short circuit the standard flying behavior

        :param forward_desired:
        :param sideways_desired:
        :param yaw_desired:
        :param height_diff_desired:
        :return:
        """
        return PreFlyReturnData(
            forward_desired=forward_desired,
            sideways_desired=sideways_desired,
            yaw_desired=yaw_desired,
            height_diff_desired=height_diff_desired
        )

    def post_fly(self, forward_desired: float = 0.0,
                 sideways_desired: float = 0.0,
                 yaw_desired: int = 0,
                 height_diff_desired: float = 0) -> None:
        """
        Method is meant to be overridden by subclasses to process flying AFTER the standard implementation.
        If this method is not overridden, it will be called and do nothing.
        Classes that override this method can operate on the resulting flying data before returning to the
        simulation.

        :param forward_desired:
        :param sideways_desired:
        :param yaw_desired:
        :param height_diff_desired:
        :return: None
        """
        return

    def fly(self, forward_desired: float = 0.0,
            sideways_desired: float = 0.0,
            yaw_desired: int = 0,
            height_diff_desired: float = 0):

        try:
            # allow for any pre-fly behavior before the standard implementation
            pre_fly_data = self.pre_fly(forward_desired, sideways_desired, yaw_desired, height_diff_desired)
            if pre_fly_data.short_circuit:
                # then the pre_fly implementation has indicated we should not continue with the normal
                # fly operation
                return
            else:
                # we should continue but use any updated values from the pre_fly hook
                forward_desired = pre_fly_data.forward_desired
                sideways_desired = pre_fly_data.sideways_desired
                yaw_desired = pre_fly_data.yaw_desired
                height_diff_desired = pre_fly_data.height_diff_desired

            # if this crazyflie is marked as the queen, let the
            # workers know how to set their motors
            if self.is_queen_bee:
                self.emitter_send(f"{forward_desired}, {sideways_desired}, {yaw_desired}, {height_diff_desired}")

            if self.is_worker_bee and self.receiver.queue_length > 0:
                # Get the received packet
                packet = self.receiver.getString()
                # Print the packet message
                # print("Received the following packet: %s" % packet)
                # Don't forget to remove the packet to get next ones
                self.receiver.nextPacket()
                queen_fly_values = packet.split(",")
                if len(queen_fly_values) == 4:
                    forward_desired = float(queen_fly_values[0].strip())
                    sideways_desired = float(queen_fly_values[1].strip())
                    yaw_desired = int(queen_fly_values[2].strip())
                    height_diff_desired = float(queen_fly_values[3].strip())

            robot_time = self.getTime()
            delta_time = robot_time - self.past_time
            if delta_time > 0.0:
                if self.first_step:
                    self.past_x_global = self.gps.getValues()[0]
                    self.past_y_global = self.gps.getValues()[1]
                    self.first_step = False

                roll = self.imu.getRollPitchYaw()[0]
                pitch = self.imu.getRollPitchYaw()[1]
                yaw = self.imu.getRollPitchYaw()[2]
                yaw_rate = self.gyro.getValues()[2]
                x_global = self.gps.getValues()[0]
                v_x_global = (x_global - self.past_x_global) / delta_time
                y_global = self.gps.getValues()[1]
                v_y_global = (y_global - self.past_y_global) / delta_time
                altitude = self.gps.getValues()[2]

                self.height_desired += height_diff_desired * delta_time

                cos_yaw = cos(yaw)
                sin_yaw = sin(yaw)
                v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
                v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

                motor_power = self.pid_controller.pid(delta_time,
                                                      forward_desired,
                                                      sideways_desired,
                                                      yaw_desired,
                                                      self.height_desired,
                                                      roll, pitch, yaw_rate,
                                                      altitude, v_x, v_y)

                self.m1_motor.setVelocity(-motor_power[0])
                self.m2_motor.setVelocity(motor_power[1])
                self.m3_motor.setVelocity(-motor_power[2])
                self.m4_motor.setVelocity(motor_power[3])

                self.past_time = self.getTime()
                self.past_x_global = x_global
                self.past_y_global = y_global

                # allow for any post processing after the standard flying implementation
                self.post_fly(forward_desired, sideways_desired, yaw_desired, height_diff_desired)

        except Exception as exc:
            print(f"Error in 'fly': {exc}")

    def init_keyboard(self, timestep):
        if self.keyboard is None:
            self.keyboard = Keyboard()

        self.keyboard.enable(timestep)

    def keyboard_handler(self):
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        key = self.keyboard.getKey()
        if not self.autonomous_mode:
            while key > 0:
                if key == Keyboard.UP:
                    forward_desired += 0.5
                elif key == Keyboard.DOWN:
                    forward_desired -= 0.5
                elif key == Keyboard.RIGHT:
                    sideways_desired -= 0.5
                elif key == Keyboard.LEFT:
                    sideways_desired += 0.5
                elif key == ord('Q'):
                    yaw_desired = + 1
                elif key == ord('E'):
                    yaw_desired = - 1
                elif key == ord('W'):
                    height_diff_desired = 0.1
                elif key == ord('S'):
                    height_diff_desired = - 0.1
                elif key == ord('A'):
                    if self.autonomous_mode is False:
                        self.autonomous_mode = True
                        print("Autonomous mode: ON")
                    else:
                        self.autonomous_mode = False
                        print("Autonomous mode: OFF")
                    time.sleep(0.5)

                key = self.keyboard.getKey()

            self.fly(forward_desired, sideways_desired, yaw_desired, height_diff_desired)
        else:
            # we are in autonmous mode so just look for the Autonmous key
            # use the default fly parameters, and let the object avoidance figure it out
            # If the actual Robot does not handle Autonmous mode then it will just sit there
            self.fly()

            if key > 0:
                if key == ord('A'):
                    if self.autonomous_mode is False:
                        self.autonomous_mode = True
                        print("Autonomous mode: ON")
                    else:
                        self.autonomous_mode = False
                        print("Autonomous mode: OFF")
                    time.sleep(0.5)

    def get_range_sensor_values(self) -> list[float | Any]:
        """
            Get range sensor values in meters.

            :return: A list of three floats representing the range sensor values. The first element is the front range, the second element is the right range, and the third element is the left
        * range.
            """
        # get range in meters
        range_front_value = self.range_front.getValue() / 1000
        range_right_value = self.range_right.getValue() / 1000
        range_left_value = self.range_left.getValue() / 1000
        range_back_value = self.range_back.getValue() / 1000
        return [range_front_value, range_back_value, range_right_value, range_left_value]

    def get_gps_position(self) -> list[float | Any]:
        return self.gps.getValues()

    def emitter_send(self, msg: str):
        self.emitter.send(msg)

    def print_instructions_to_console(self):
        if self.is_queen_bee:
            print("====== Controls =======\n\n")

            print(" The Crazyflie can be controlled from your keyboard!\n")
            print(" All controllable movement is in body coordinates\n")
            print("- Use the up, back, right and left button to move in the horizontal plane\n")
            print("- Use Q and E to rotate around yaw\n ")
            print("- Use W and S to go up and down\n ")
            print("- Use W and S to go up and down\n ")
        elif self.is_worker_bee:
            print("====== Worker Drone =======\n\n")


class DroneBlocksWebotCrazyflieObstacleAvoidanceRobot(DroneBlocksWebotCrazyflieRobot):
    class FlyingState(Enum):
        FORWARD = 0
        TURN = 1

    def __init__(self, lower_distance_threshold: float = 0.8, upper_distance_threshold: float = 1.8, forward_speed: float = 0.5):
        super().__init__()
        self.fly_state = DroneBlocksWebotCrazyflieObstacleAvoidanceRobot.FlyingState.FORWARD
        self.yaw_velocity = 1
        self.lower_distance_threshold =lower_distance_threshold
        self.upper_distance_threshold = upper_distance_threshold
        self.forward_speed = forward_speed

    def pre_fly(self, forward_desired: float = 0.0,
                sideways_desired: float = 0.0,
                yaw_desired: int = 0,
                height_diff_desired: float = 0) -> PreFlyReturnData:

        if self.is_autonomous_flight():
            range_sensor_values = self.get_range_sensor_values()

            # set the proper velocities and yaw
            if self.fly_state == DroneBlocksWebotCrazyflieObstacleAvoidanceRobot.FlyingState.FORWARD:
                yaw_desired = 0
                forward_desired = self.forward_speed

                # check the front range sensor value.
                if range_sensor_values[0] < self.lower_distance_threshold:
                    self.fly_state = DroneBlocksWebotCrazyflieObstacleAvoidanceRobot.FlyingState.TURN
                    # before turning pick a random left/right turn ( yaw ) values
                    # and stop flying forward
                    yaw_desired = random.choice([-1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1])
                    forward_desired = 0.0

                self.yaw_velocity = yaw_desired

            elif self.fly_state == DroneBlocksWebotCrazyflieObstacleAvoidanceRobot.FlyingState.TURN:
                # if we are turning... override the yaw_desired with the selected yaw_velocity when entering the TURN state
                yaw_desired = self.yaw_velocity
                if range_sensor_values[0] > self.upper_distance_threshold:
                    if random.randint(0,10) <= 6:
                        # n% of the time go foward immediately, but 100-n% keep rotating a little longer to cover more area
                        self.fly_state = DroneBlocksWebotCrazyflieObstacleAvoidanceRobot.FlyingState.FORWARD
                        self.yaw_velocity = 0

        rtn_data = PreFlyReturnData(
            short_circuit=False,
            forward_desired=forward_desired,
            sideways_desired=sideways_desired,
            yaw_desired=yaw_desired,
            height_diff_desired=height_diff_desired
        )
        return rtn_data

    def print_instructions_to_console(self):
        print("\n")

        if self.is_worker_bee:
            print("====== Worker Drone =======\n\n")
        else:
            print("====== Controls =======\n\n")

            print(" The Crazyflie can be controlled from your keyboard!\n")
            print(" All controllable movement is in body coordinates\n")
            print("- Use the up, back, right and left button to move in the horizontal plane\n")
            print("- Use Q and E to rotate around yaw\n ")
            print("- Use W and S to go up and down\n ")
            print("- Use W and S to go up and down\n ")
            print("- Use A for Autonomous Object Avoidance Mode \n ")


class DroneBlocksWebotCrazyfliePathDisplayRobot(DroneBlocksWebotCrazyflieObstacleAvoidanceRobot):

    def __init__(self):
        super().__init__()
        try:
            self.display = self.getDevice("display")
            print(print([attr for attr in dir(self.display) if not attr.startswith('__')]))
            print(f"W,H: {self.display.getWidth()}, {self.display.getHeight()}")
            self.display_width = self.display.getWidth()
            self.display_height = self.display.getHeight()
        except Exception as e:
            self.display = None
            self.display_width = None
            self.display_height = None

        self.GROUND_X = 3.0
        self.GROUND_Y = 3.0
        self.LIGHT_GRAY = 0x505050
        self.RED = 0xBB2222
        self.GREEN = 0x22BB11
        self.BLUE = 0x2222BB
        self.tracking_opacity = 0.5

    def post_fly(self, forward_desired: float = 0.0,
                sideways_desired: float = 0.0,
                yaw_desired: int = 0,
                height_diff_desired: float = 0) -> None:

        x_global = self.gps.getValues()[0]*-1
        y_global = self.gps.getValues()[1]
        z_global = self.gps.getValues()[2]

        if self.display:
            self.display.setOpacity(self.tracking_opacity)
            self.display.setColor(self.RED)
            display_x = self.display_width - self.display_width * (x_global + self.GROUND_X / 2) / self.GROUND_X
            display_y = self.display_height - self.display_height * (y_global + self.GROUND_Y / 2) / self.GROUND_Y
            print(f"X/Y/Z: {x_global}, {y_global}, {z_global} | display_x, display_y: {display_x}, {display_y}")
            print("#################")

            self.display.fillOval(display_x, display_y, 4, 4)
        else:
            print(f"X/Y/Z: {x_global}, {y_global}, {z_global}")
            print("#################")
