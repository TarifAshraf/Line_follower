from controller import Robot

TIME_STEP = 64
robot = Robot()

irNames = ['ir_left_corner', 'ir_left', 'ir_mid', 'ir_right', 'ir_right_corner']
ir = []
for i in range(5):
    ir.append(robot.getDevice(irNames[i]))
    ir[i].enable(TIME_STEP)

# dsNames = ['distanceRight', 'distanceLeft']
# for i in range(2):
    # ds.append(robot.getDevice(dsNames[i]))
    # ds[i].enable(TIME_STEP)

camera = robot.getDevice('cameraSensor')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# gps = robot.getDevice('gpsSensor')
# gps.enable(TIME_STEP)

wheels = []
wheelsNames = ['frontLeftWheel', 'frontRightWheel', 'backLeftWheel', 'backRightWheel']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

speed = 2
k_p, k_i, k_d = 1.5, 0.015, 0.3
I = 0
previousError = 0

while robot.step(TIME_STEP) != -1:

    ir_flag = [0, 0, 0, 0, 0]
    ir_error = [-1, -1, 0, 1, 1]
    for i in range(5):
        if int(ir[i].getValue()) < 1000.0:
            ir_flag[i] = 0
        else:
            ir_flag[i] = 1

    error = 0
    for i in range(5):
        error += ir_flag[i] * ir_error[i]
    
    P = error
    I = I + error
    D = error-previousError
    pid = (k_p*P) + (k_i*I) + (k_d*D)
    
    # pid = ((k_p*error)+k_i+(k_d*error*error))/error
    
    leftSpeed = speed + pid
    rightSpeed = speed - pid

    if rightSpeed > speed:
        leftSpeed = 0
    
    if leftSpeed > speed:
        rightSpeed = 0

    if leftSpeed < 0:
        leftSpeed = 0
  
    if rightSpeed < 0:
        rightSpeed = 0

    previousError = error
 
    # print(ir_flag)
    # print(error)
    # print(pid)
    # print(leftSpeed, rightSpeed)
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)