import cv2
import numpy as np
import cozmo
from cozmo.util import degrees, speed_mmps, #distance_mm
import asyncio
from Detect_Tags import DetectTags


def readcard(robot: cozmo.robot.Robot):
    dtHolder = DetectTags()
    if dtHolder.check_image() != -1:
        if dtHolder.check_image() == 4:
            action = robot.turn_in_place(degrees(90), speed_mmps(50))
            action.wait_for_completed()

        return


def cozmo_program(robot: cozmo.robot.Robot):
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cube = None
    try:
        cube = robot.world.wait_for_observed_light_cube(timeout=30)
        print("Found cube", cube)

    except asyncio.TimeoutError:
        print("Didn't find a cube")

    finally:
        look_around.stop()

    if cube is None:
        robot.say_text("Oh no I don't see a cube").wait_for_completed()
        return

cozmo.run_program(readcard)
#if __name__ == '__main__':
    #readcard()


