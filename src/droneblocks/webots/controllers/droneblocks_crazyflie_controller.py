"""crazyflie_controller controller."""

import random
import sys

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from droneblocks.webots.robots.droneblocks_webot_crazyflie import DroneBlocksWebotCrazyflieRobot


class DroneBlocksWebotsCrazyflieController():

    def __init__(self, crazyflie: DroneBlocksWebotCrazyflieRobot | None = None) -> None:
        self.crazyflie = crazyflie

    def set_crazyflie_robot(self, robot: DroneBlocksWebotCrazyflieRobot):
        self.crazyflie = robot

    def run(self) -> None:
        if self.crazyflie is None:
            raise Exception("Crazyflie not initialized")

        crazyflie = self.crazyflie
        # create the Robot instance.
        timestep = crazyflie.initialize()

        print("\n")

        if self.crazyflie.is_queen_bee:
            print("====== Controls =======\n\n")

            print(" The Crazyflie can be controlled from your keyboard!\n")
            print(" All controllable movement is in body coordinates\n")
            print("- Use the up, back, right and left button to move in the horizontal plane\n")
            print("- Use Q and E to rotate around yaw\n ")
            print("- Use W and S to go up and down\n ")
        elif self.crazyflie.is_worker_bee:
            print("====== Worker Drone =======\n\n")

        # 0 - fly forward
        # 1 - turn ( until forward sensor is >= 2 )
        fly_state = 0
        yaw_velocity = 1

        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        while crazyflie.step(timestep) != -1:
            crazyflie.keyboard_handler()
            if crazyflie.is_autonomous_flight():
                range_sensor_values = crazyflie.get_range_sensor_values()

                # print(f"GPS Position: {crazyflie.get_gps_position()}")

                # set the proper velocities and yaw
                sideways_velocity = 0.0
                height_diff = 0.0
                if fly_state == 0:
                    # fly foward state
                    yaw_velocity = 0
                    foward_velocity = 0.5

                    if range_sensor_values[0] < 0.8:
                        fly_state = 1
                        # before turning pick a random left/right turn ( yaw ) values
                        yaw_velocity = random.choice([-1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1])

                elif fly_state == 1:
                    # turn state

                    # yaw_velocity = 1
                    foward_velocity = 0.0

                    if range_sensor_values[0] > 1.8:
                        fly_state = 0

                crazyflie.fly(forward_desired=foward_velocity, sideways_desired=sideways_velocity,
                              yaw_desired=yaw_velocity,
                              height_diff_desired=height_diff)

                # print(f"Range Values: {range_sensor_values})")


if __name__ == '__main__':

    # for i in range(1, len(sys.argv)):
    #     print("argv[%i]=%s" % (i, sys.argv[i]))
    #
    crazyflie_robot = DroneBlocksWebotCrazyflieRobot()
    # print(f"sys.argv[0]={sys.argv[0]}")
    # print(f"sys.argv[1]={sys.argv[1]}")
    if len(sys.argv) > 1:
        if sys.argv[1] == "queen":
            crazyflie_robot.set_queen_bee(True)
        elif sys.argv[1] == "worker":
            crazyflie_robot.set_worker_bee(True)
    crazyflie_controller = DroneBlocksWebotsCrazyflieController(crazyflie=crazyflie_robot)

    crazyflie_controller.run()
