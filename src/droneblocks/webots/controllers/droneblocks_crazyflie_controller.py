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

        crazyflie.print_instructions_to_console()

        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        while crazyflie.step(timestep) != -1:
            crazyflie.keyboard_handler()


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
