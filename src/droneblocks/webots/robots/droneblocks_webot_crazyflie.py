import time

from controller import Robot
from droneblocks.crazyflie.pid_controller import PIDController
from math import cos, sin
from typing import Any
from controller import Keyboard


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
                height_diff_desired: float = 0) -> bool:
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
        return True

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
            if not self.pre_fly(forward_desired, sideways_desired, yaw_desired, height_diff_desired):
                return

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