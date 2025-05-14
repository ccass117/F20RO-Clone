from controller import Robot

def run_robot(robot: Robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    prox_sensors = []
    for i in range(8):
        sensor = robot.getDevice(f'ps{i}')
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    light_sensors = []
    for i in range(8):
        sensor = robot.getDevice(f'ls{i}')
        sensor.enable(timestep)
        light_sensors.append(sensor)

    def parse_message(message):
        values = list(map(float, message.split(",")))
        return tuple(values)

    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)

    reward_zone = None

    while robot.step(timestep) != -1:
        left_wall = prox_sensors[5].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80
        right_wall = prox_sensors[2].getValue() > 80
        right_corner = prox_sensors[1].getValue() > 80
        front_wall = prox_sensors[0].getValue() > 80 or prox_sensors[7].getValue() > 80

        left_speed = max_speed
        right_speed = max_speed

        emitter = robot.getDevice('emitter')
        emitter.setChannel(20)

        light_on = False
        light_threshold = 5000
        if any(light_sensors[i].getValue() >= light_threshold for i in [1, 2]):
            light_on = True

        light_status_message = "light_on" if light_on else "light_off"
        emitter.send(light_status_message.encode('utf-8'))


        print("Light sensor readings:", [sensor.getValue() for sensor in light_sensors])

        if receiver.getQueueLength() > 0:
            message = receiver.getString()
            robot_data, reward_data = message.split("|")
            robot_position = parse_message(robot_data)
            reward_zone = parse_message(reward_data)
            print(f"Received robot position: {robot_position}")
            print(f"Received reward zone: {reward_zone}")
            receiver.nextPacket()

        if left_wall and right_wall:
            if prox_sensors[5].getValue() > prox_sensors[2].getValue():
                left_speed = max_speed / 2
                right_speed = max_speed
            elif prox_sensors[5].getValue() < prox_sensors[2].getValue():
                    left_speed = max_speed
                    right_speed = max_speed / 2

        if light_on:
            print("Received reward zone: light on")
            if front_wall:
                print("Turn right in place")
                left_speed = max_speed
                right_speed = -max_speed
            else:
                if left_wall:
                    print("Follow right wall")
                    left_speed = max_speed
                    right_speed = max_speed
                    if prox_sensors[5].getValue() < prox_sensors[6].getValue():
                        print("Adjusting to straighten out from left wall")
                        left_speed = max_speed
                        right_speed = max_speed / 2
                    elif prox_sensors[5].getValue() > prox_sensors[6].getValue():
                        print("Adjusting to straighten out from left wall")
                        left_speed = max_speed / 2
                        right_speed = max_speed
                else:
                    print("Turn left")
                    left_speed = max_speed / 8
                    right_speed = max_speed

                if right_corner:
                    print("Turn left")
                    left_speed = max_speed / 8
                    right_speed = max_speed

        else: 
            print("Received reward zone: light off")
            if front_wall:
                print("Turn left in place")
                left_speed = -max_speed
                right_speed = max_speed
            else:
                if right_wall:
                    print("Follow left wall")
                    left_speed = max_speed
                    right_speed = max_speed
                    if prox_sensors[2].getValue() < prox_sensors[1].getValue():
                        print("Adjusting to straighten out from right wall")
                        left_speed = max_speed / 2
                        right_speed = max_speed
                    elif prox_sensors[2].getValue() > prox_sensors[1].getValue():
                        print("Adjusting to straighten out from right wall")
                        left_speed = max_speed
                        right_speed = max_speed / 2
                else:
                    print("Turn right")
                    left_speed = max_speed
                    right_speed = max_speed / 8

                if left_corner:
                    print("Turn right")
                    left_speed = max_speed
                    right_speed = max_speed / 8
                
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        if reward_zone and (
            (reward_zone == (0.3, 0.05, 0.15) and abs(reward_zone[0] - robot_position[0]) < 0.05 and abs(reward_zone[2] - robot_position[2]) < 0.05) or
            (reward_zone == (-0.3, 0.05, -0.15) and abs(reward_zone[0] - robot_position[0]) < 0.05 and abs(reward_zone[2] - robot_position[2]) < 0.05)
        ):
            print("Reached reward zone, stopping")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break

if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)
