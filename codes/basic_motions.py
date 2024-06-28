from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import time

def main():
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.set_ee_pose_components(x=0.2, z=0.3, pitch=1.5)

    while True:
        user_direction = input("x, y, z, r(roll), p(pitch), y(yaw), esc\nAnd add +/-:\t")

        if user_direction == "x+":
            bot.arm.set_ee_cartesian_trajectory(x=0.05)
        elif user_direction == "x-":
            bot.arm.set_ee_cartesian_trajectory(x=-0.05)

        elif user_direction == "y+":
            bot.arm.set_ee_cartesian_trajectory(y=0.05)
        elif user_direction == "y-":
            bot.arm.set_ee_cartesian_trajectory(y=-0.05)

        elif user_direction == "z+":
            bot.arm.set_ee_cartesian_trajectory(z=0.05)
        elif user_direction == "z-":
            bot.arm.set_ee_cartesian_trajectory(z=-0.05)

        elif user_direction == "r+":
            bot.arm.set_ee_cartesian_trajectory(roll=0.25) # 14 degrees
        elif user_direction == "r-":
            bot.arm.set_ee_cartesian_trajectory(roll=-0.25)

        elif user_direction == "p+":
            bot.arm.set_ee_cartesian_trajectory(pitch=0.25)
        elif user_direction == "p-":
            bot.arm.set_ee_cartesian_trajectory(pitch=-0.25)

        elif user_direction == "ya+":
            bot.arm.set_ee_cartesian_trajectory(yaw=0.25)
        elif user_direction == "ya-":
            bot.arm.set_ee_cartesian_trajectory(yaw=-0.25)

        elif user_direction == "esc":
            break
        else:
            print("try again")
            pass
        # time.sleep(0.6)

    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
