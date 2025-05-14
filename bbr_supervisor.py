from controller import Supervisor

supervisor = Supervisor()
TIME_STEP = 64

light_node = supervisor.getFromDef("Light")

if light_node is None:
    raise RuntimeError("Light node not found")

intensity_field = light_node.getField("intensity")
if intensity_field is None:
    raise RuntimeError("Light node does not have an intensity field")

def is_light_on():
    return intensity_field.getSFFloat() > 0.1

robot_node = supervisor.getFromDef("Controller")
if robot_node is None:
    raise RuntimeError("Robot node not found")
robot_position_field = robot_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    light_status = "on" if is_light_on() else "off"
    print(f"Light is {light_status}")

    robot_position = robot_position_field.getSFVec3f()
    print(f"Position: {robot_position}")
    receiver = supervisor.getDevice('receiver')
    receiver.enable(TIME_STEP)
    receiver.setChannel(20)

    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        if message == "light_on":
            reward_zone = [0.3, 0.05, 0.15] 
        else:
            reward_zone = [-0.3, 0.05, -0.15]

        print(f"Reward zone coordinates: {reward_zone}")
