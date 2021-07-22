#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
# import numpy as np
import sim

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    # Camera handles
    _,cam = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_camera',
                                        sim.simx_opmode_oneshot_wait)
    sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    sim.simxReadVisionSensor(clientID, cam, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, cam

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------

# def getImage(clientID, hRobot):
#     img = []
#     err,r,i = sim.simxGetVisionSensorImage(clientID, hRobot[2], 0,
#                                             sim.simx_opmode_buffer)

#     if err == sim.simx_return_ok:
#         img = np.array(i, dtype=np.uint8)
#         img.resize([r[1],r[0],3])
#         img = np.flipud(img)
#         img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

#     return err, img

# --------------------------------------------------------------------------

def getImageBlob(clientID, hRobot):
    rc,ds,pk = sim.simxReadVisionSensor(clientID, hRobot[2],
                                         sim.simx_opmode_buffer)
    blobs = 0
    coord = []
    if rc == sim.simx_return_ok and pk[1][0]:
        blobs = int(pk[1][0])
        offset = int(pk[1][1])
        for i in range(blobs):
            coord.append(pk[1][4+offset*i])
            coord.append(pk[1][5+offset*i])

    return blobs, coord

# --------------------------------------------------------------------------

def avoid(sonar, direction, integral, P_anterior, blobs, error):
    Kp = 0.5
    Ki = 0.3
    Kd = 0.1
    velocidad = 1
    steer = 0
    minIdx = sonar.index(min(sonar))
    P = 0

    if blobs:
        if (sonar[3] > 0.12 and error < 0) or (sonar[4] > 0.12 and error > 0):
            sensor_loc=[-4,-3,-2,-1,1,2,3,4]
    
            if (direction == 0):
                for i in range(0,8):
                    P += sensor_loc[i] * sonar[i]
                D = P - P_anterior
                U = Kp * P - Ki * integral + Kd * D
            else:
                for i in range(0,8):
                    P+=sensor_loc[i] * sonar[i+8]
                D = P - P_anterior
                U = Kp * P + Ki * integral + Kd * D
                velocidad = -1 * velocidad 
                    
            lspeed, rspeed = velocidad + U, velocidad - U
        else:
            lspeed = -2
            rspeed = -2


    else:
        if minIdx in [1, 2, 3, 9, 10, 11]:
            steer = math.pi * (-1) ** direction
        elif minIdx in [4, 5, 6, 12, 13, 14]:
            steer = -math.pi * (-1) ** direction
        else:
            steer = (sonar[15] + sonar[7] - sonar[0] - sonar[8]) * math.pi * (-1) ** direction 
            if sonar[15] + sonar[0] > 0.8 and min(sonar[7:9]) < 0.5:
                steer = -math.pi / 2
            elif sonar[7] + sonar[8] > 0.8 and min([sonar[15],sonar[0]]) < 0.5:
                steer = math.pi / 2
            elif (direction == 0 and sum(sonar[2:6]) < 2) or (direction == 1 and sum(sonar[10:14]) < 2):
                if steer > 0:
                    steer = math.pi
                else:
                    steer = -math.pi
             

        if direction == 0:
            v = min(sonar[2:6])
            if v < .1:
                v = min(sonar[10:14])
                direction = 1
            
        else:
            v = min(sonar[10:14])
            if v < .1:
                v = min(sonar[2:6])
                direction = 0

        v *= 4
    
        lspeed = (v + Kp * steer) * (-1) ** direction
        rspeed = (v - Kp * steer) * (-1) ** direction

    return lspeed, rspeed, direction, integral, P

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        # Definimos 4 estados posibles
        AVOID = 0
        SEARCH = 1 # Busca la pelota. Gira 360º y después avanza buscando la pelota
        APPROACH = 2 # Acercarse a la pelota hasta que el sonar la detecte cerca
        ESCORT = 3 # Seguir a la pelota sin acercarse ni alejarse demasiado

        FWD = 0
        BWD = 1

        # Parámetros obtenidos de manera experimental:
        steps_360 = 120
        Kc = 3.5
        Pc = 10


        # Inicialización
        cur_state = SEARCH
        step_count = 0
        error = 0
        integral = 0
        d = 0
        prevError = 0
        direction = FWD
        new_state = SEARCH

        PIDKp = 0.6 * Kc
        PIDKi = 2 * PIDKp / Pc
        PIDKd = PIDKp * Pc / 8


        while sim.simxGetConnectionId(clientID) != -1:


            # Perception
            sonar = getSonar(clientID, hRobot)
            # print '### s', sonar
            #print(sonar)

            blobs, coord = getImageBlob(clientID, hRobot)
            #print('###  ', blobs, coord)

            if blobs:
                error = (coord[0] - 0.5) # Paso coord x de [0, 1] a [-1, 1]

            # Update internal state
            if min(sonar) < .3 and not (blobs and ((sonar[3] > 0.12 and error < 0) or (sonar[4] > 0.12 and error > 0))):
                if new_state != AVOID:
                    new_state = AVOID
                else:
                    integral = 0
            else:
                if blobs:
                    if error < 0:
                        if sonar[3] < 1:
                            new_state = ESCORT
                            direction = FWD
                        else:
                            new_state = APPROACH
                            direction = FWD
                    else:
                        if sonar[4] < 1:
                            new_state = ESCORT
                            direction = FWD
                        else:
                            new_state = APPROACH
                            direction = FWD
                else:
                    new_state = SEARCH

                if new_state != cur_state:
                    step_count = 0
                    integral = 0
                    d = 0
                    cur_state = new_state

            if new_state == AVOID:
                lspeed, rspeed, direction, integral, d = avoid(sonar, direction, integral, d, blobs, error)


            elif cur_state == SEARCH:
                # En primer lugar giramos sin movernos del sitio buscando la pelota y también cada 200 pasos hacemos un giro
                if step_count % 200 < steps_360:
                    # Pelota a la derecha
                    if error >= 0:
                        lspeed = 1
                        rspeed = -1
                    else:
                        lspeed = -1
                        rspeed = 1
                else:
                    lspeed = 4 * (-1) ** direction
                    rspeed = 4 * (-1) ** direction

                step_count += 1

            else: 
                prevError = error
                
                PIDtv = PIDKp * error + PIDKi * integral + PIDKd * (error - prevError)
                tv = max(-1, min(1, PIDtv)) # Limit the command that will be sent

                if cur_state == APPROACH:
                    if abs(error) < 0.1:
                        fv = 1.5 # Máxima velocidad
                    else:
                        fv = abs(1.5 * error)
                else: # ESCORTING
                    fv = 1.2


                # Integrator anti-windup (clamping method)
                if PIDtv == tv or PIDtv * error < 0:
                    integral += error

                lspeed = tv + fv
                rspeed = fv - tv


                

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
