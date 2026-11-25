#!/usr/bin/env python

"""Read joint positions and torques through Flexiv RDK without enabling the robot."""

import argparse
import time

import flexivrdk
import spdlog


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot, e.g. Rizon4s-123456",
    )
    args = argparser.parse_args()

    logger = spdlog.ConsoleLogger("RDK state reader")
    logger.info("Connecting in read-only mode; the robot will not be enabled")

    try:
        robot = flexivrdk.Robot(args.robot_sn)

        while True:
            states = robot.states()
            print(f"q:   {['%.3f' % value for value in states.q]}")
            print(f"tau: {['%.3f' % value for value in states.tau]}", flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Exiting")
    except Exception as exception:
        logger.error(str(exception))
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
