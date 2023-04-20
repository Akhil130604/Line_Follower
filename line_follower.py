import time
import board
import digitalio
import analogio
import pwmio
sensor_pins=[analogio.AnalogIn(getattr(board,f"A{i}"))for i in range(7)]
button1=digitalio.DigitalInOut(board.D8)
motor_pins = [board.D2, board.D3, board.D4, board.D5]
motor_direction=[digitalio.Direction.OUTPUT]*4
motor_inputs = [digitalio.DigitalInOut(pin) for pin in motor_pins]
for motor_input in motor_inputs:
    motor_input.direction = digitalio.Direction.OUTPUT
ENB=pwmio.PWMOut(board.D0, frequency=1000, duty_cycle=0)
ENA=pwmio.PWMOut(board.D1, frequency=1000, duty_cycle=0)
led=digitalio.DigitalInOut(board.D6)
led.direction=digitalio.Direction.OUTPUT
button=digitalio.DigitalInOut(board.D7)
lowest=[65535]*len(sensor_pins)
highest,Sum,avg,threshold_val=[0]*len(sensor_pins),[0]*len(sensor_pins),[0]*len(sensor_pins),[0]*len(sensor_pins)
def caliberation():
    ENA.duty_cycle=int(35000)
    ENB.duty_cycle=int(35000)
    motor_inputs[0].value=False
    motor_inputs[1].value=True
    motor_inputs[2].value=True
    motor_inputs[3].value=False
    led.value=True
    for i in range(1000):
        sensor_values=[pin.value for pin in sensor_pins]
        time.sleep(0.01)
        print(sensor_values)
        for j,value in enumerate(sensor_values):
            if(value<lowest[j]):
                lowest[j]=value
        for j,value in enumerate(sensor_values):
            if(value>highest[j]):
                highest[j]=value
    for j in range(len(sensor_pins)):
        Sum[j]=lowest[j]+highest[j]
        avg[j]=Sum[j]/2
    led.value=False
    ENA.duty_cycle=int(0)
    ENB.duty_cycle=int(0)
    motor_inputs[0].value=False
    motor_inputs[1].value=False
    motor_inputs[2].value=False
    motor_inputs[3].value=False
    return(avg[j])
def left():
    ENA.duty_cycle=int(65534)
    ENB.duty_cycle=int(65534)
    motor_inputs[0].value=False
    motor_inputs[1].value=True
    motor_inputs[2].value=True
    motor_inputs[3].value=False
def right():
    ENA.duty_cycle=int(65534)
    ENB.duty_cycle=int(65534)
    motor_inputs[0].value=True
    motor_inputs[1].value=False
    motor_inputs[2].value=False
    motor_inputs[3].value=True
def linefollow(speed):
    error,P,I,D,PIDvalue,previousError = 0,0,0,0,0,0
    error = (sensor_value[2] - sensor_value[4])
    P = error
    I = I + error
    D = error - previousError

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D)
    previousError = error

    lsp = speed - PIDvalue
    rsp = speed + PIDvalue

    if(lsp > 65530):
        lsp = 65530
    if (lsp < 0): 
        lsp = 0
    if (rsp > 65530):
        rsp = 65530
    if (rsp < 0): 
        rsp = 0;
    ENA.duty_cycle=int(rsp)
    ENB.duty_cycle=int(lsp)
    motor_inputs[0].value=False
    motor_inputs[1].value=True
    motor_inputs[2].value=False
    motor_inputs[3].value=True
    
speed=65530
Kp,Kd,Ki=0,0,0
while True:
    but1val=button.value
    but2val=button1.value
    if(but1val==True):
        caliberation()
        for i in range(7):
            threshold_val[i]=avg[i]
    if(but2val==True):
        while True:
            sensor_value=[pin.value for pin in sensor_pins]
            time.sleep(0.1)
            #BBWW
            if(sensor_value[0]>threshold_val[0] and sensor_value[1]>threshold_val[1] and sensor_value[5]<threshold_val[5] and sensor_value[6]<threshold_val[6]):
                left()
            #BWWW
            elif(sensor_value[0]>threshold_val[0] and sensor_value[1]<threshold_val[1] and sensor_value[5]<threshold_val[5] and sensor_value[6]<threshold_val[6]):
                left()
            #WBWW
            elif(sensor_value[0]<threshold_val[0] and sensor_value[1]>threshold_val[1] and sensor_value[5]<threshold_val[5] and sensor_value[6]<threshold_val[6]):
                left()
            #WWBB
            elif(sensor_value[0]<threshold_val[0] and sensor_value[1]<threshold_val[1] and sensor_value[5]>threshold_val[5] and sensor_value[6]>threshold_val[6]):
                right()
            #WWBW
            elif(sensor_value[0]<threshold_val[0] and sensor_value[1]<threshold_val[1] and sensor_value[5]>threshold_val[5] and sensor_value[6]<threshold_val[6]):
                right()
            #WWWB
            elif(sensor_value[0]<threshold_val[0] and sensor_value[1]<threshold_val[1] and sensor_value[5]<threshold_val[5] and sensor_value[6]>threshold_val[6]):
                right()
            else:
                Kp = 0.0006 * (1000 - sensor_value[3]);
                Kd = 10 * Kp;
                Ki = 0.0001;
                linefollow(speed);
            
time.sleep(0.01)