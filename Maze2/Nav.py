import asyncio
from mavsdk import System
from mavsdk.offboard import (PositionNedYaw, OffboardError)

async def run():
    drone = System()
    print("--Establishing Connection")
    await drone.connect()

    if drone.core.connection_state():
        print("--Drone Connected")

        await drone.action.arm()

        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()

        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.5, -1.5, 0.0))
        await asyncio.sleep(5)

        await drone.offboard.set_position_ned(PositionNedYaw(-7.0, 0.5, -1.5, 180.0))
        await asyncio.sleep(5)

        await drone.offboard.set_position_ned(PositionNedYaw(-7.0, -16.0, -1.5, 270.0))
        await asyncio.sleep(7)

        await drone.offboard.set_position_ned(PositionNedYaw(3.0, -16.0, -1.5, 0.0))
        await asyncio.sleep(5)

        await drone.offboard.set_position_ned(PositionNedYaw(3.0, -22.0, -1.5, 270.0))
        await asyncio.sleep(5)

        await drone.offboard.stop()

        await drone.action.land()

        print("--Mission Complete")


if __name__ == "__main__":
    asyncio.run(run())