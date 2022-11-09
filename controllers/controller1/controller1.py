from controller import Robot

TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ['distanceRight', 'distanceLeft']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

camera = robot.getDevice('cameraSensor')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

gps = robot.getDevice('gpsSensor')
gps.enable(TIME_STEP)
# print(gps.getCoordinateSystem())

wheels = []
wheelsNames = ['frontLeftWheel', 'frontRightWheel', 'backLeftWheel', 'backRightWheel']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

avoidObstacleCounter = 0
rflag = False
lflag = False
while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        if lflag:
            leftSpeed = 1.0
            rightSpeed = -1.0
        elif rflag:
            leftSpeed = -1.0
            rightSpeed = 1.0
        if avoidObstacleCounter == 0:
            lflag = False
            rflag = False
    else:
        for i in range(2):
            if ds[i].getValue() < 1000.0:
                leftSpeed = 0.0
                rightSpeed = 0.0
                avoidObstacleCounter = 100
                if i == 0:
                    rflag = True
                if i == 1:
                    lflag = True
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)