#BASAD

from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import radians, degrees, sin, cos

hub = InventorHub()

#hardware definition
x_motor=Motor(port=Port.F, positive_direction=Direction.COUNTERCLOCKWISE) # x is along the track
y_motor=Motor(port=Port.B, positive_direction=Direction.COUNTERCLOCKWISE) # y is up and down
z_motor=Motor(port=Port.E) # z is extend forward and back
tx_motor=Motor(port=Port.C, positive_direction=Direction.COUNTERCLOCKWISE) # tx rotates the gripper
gripper_motor=Motor(port=Port.A) # gripper catches the candle

# gear ratios (Motor degrees --> mm or deg)
x_ratio=1/(12/360*32/10)
y_ratio=1/(20/12*40/27/360)
z_ratio=x_ratio
tx_ratio=1/(8/28)
gripper_ratio=1/(1/20)

# ratio dictionary
ratios={
    x_motor: x_ratio,
    y_motor: y_ratio,
    z_motor: z_ratio,
    tx_motor: tx_ratio,
    gripper_motor: gripper_ratio
}

# default motor speed (°/sec)
speeds={
    x_motor: 700,
    y_motor: 700,
    z_motor: 100,
    tx_motor: 500,
    gripper_motor: 500
}


#starting positions 
x_start=0.0 # leftmost corner
y_start=0.0 # bottom
z_start=0.0 # arm fully retracted
tx_start=0.0 # gripper parallel to ground
grip_start=0.0 #gripper plates parallel to each other

# position limits dictionary
position_limits={
    x_motor: (0, 286),
    y_motor: (0, 32),
    z_motor: (0, 48),
    tx_motor: (-45, 45 ),
    gripper_motor: (-30, 20)
}
 
# positions dictionary
positions={
    x_motor: x_start,
    y_motor: y_start,
    z_motor: z_start,
    tx_motor: tx_start,
    gripper_motor: grip_start
}

# more parameters
gripper_closed_pos=15
gripper_open_pos=-3

# geometry_parameters
candle_above_gripper_mm=70
gripper_y_offset_mm=28
gripper_z_offset_mm=52


# misc
char_to_motor={ # helps translate movement order commands
    'x': x_motor,
    'y': y_motor,
    'z': z_motor,
    't': tx_motor
}


def check_out_of_limits(motor: Motor, target_pos: float):
    # check if target position is illegal because it sends the robot out of limits:
    if target_pos<position_limits[motor][0] or target_pos>position_limits[motor][1]:
        raise ValueError(
            f"Motor {motor} move would exceed limits: {position_limits[motor][0]} to {position_limits[motor][1]}, requested end position = {target_pos}")
    #if all is ok
    return False

def relative_move(motor: Motor, move_by: float,):
    amount_motor_degrees=int(move_by*ratios[motor]) # calculate required amount
    end_position=positions[motor]+move_by # check where this action will take the motor
    check_out_of_limits(motor, end_position)
    motor.run_angle(speeds[motor], amount_motor_degrees,wait=False) # perform motion
    positions[motor]=end_position # update position of selected axis



def absolute_move(motor: Motor, target: float):
    current_pos=positions[motor]
    amount_to_move=target-current_pos # calculate how much to move to target
    relative_move(motor, amount_to_move) # move and update position

def go_home():
    # returns all motors to start position
    absolute_move(tx_motor,tx_start)
    wait_for_motors([tx_motor])
    absolute_move(z_motor,z_start)
    absolute_move(y_motor, y_start)
    wait_for_motors([y_motor,z_motor])
    absolute_move(x_motor,x_start)
    wait_for_motors([x_motor])
    absolute_move(gripper_motor, grip_start)
    wait_for_motors([gripper_motor])

def wait_for_motors(motors: list[Motor], inclusive=True):
    # wait for either all motors in list to complete motion (inclusive = True)
    # or for the first of the list (inclusive = False)
    
    while True:
        motors_done = [motor.done() for motor in motors]

        if inclusive:
            # all must be done
            if all(motors_done):
                break
        else:
            # first one finished is enough
            if any(motors_done):
                break

        wait(10)  # small delay to avoid busy-waiting
    

def candle_offsets(target_angle: float):
    # calculates the y and z offsets of the candle tip from the tx joint center
    dz_candle=sin(radians(target_angle))*(gripper_y_offset_mm+candle_above_gripper_mm)+cos(radians(target_angle))*gripper_z_offset_mm

    dy_candle=cos(radians(target_angle))*(gripper_y_offset_mm+candle_above_gripper_mm)-sin(radians(target_angle))*gripper_z_offset_mm

    return dy_candle, dz_candle

def candle_tip_inverse_kinematics(tip_x: float, tip_y: float, tip_z: float, tip_tx: float):
    # calculates where the robot needs to go for the candle tip to arrive to target position
    dy_candle, dz_candle=candle_offsets(tip_tx) # calculate the y, z position compensations for the candle tilt

    #returns dictionary of target_positions:
    target_positions={
        x_motor: tip_x,
        y_motor: tip_y-dy_candle,
        z_motor: tip_z-dz_candle,
        tx_motor: tip_tx
    }
    print (target_positions)
    return target_positions

def candle_tip_to(tip_x: float, tip_y: float, tip_z: float, tip_tx: float, move_order=['x','zy','t']):
    # absolute move the candle tip to target position
    # calculate IK for required robot position
    target_robot_positions=candle_tip_inverse_kinematics(tip_x, tip_y, tip_z, tip_tx)

    # check if any of the target positions is out of limits
    check_out_of_limits(x_motor, target_robot_positions[x_motor])
    check_out_of_limits(y_motor, target_robot_positions[y_motor])
    check_out_of_limits(z_motor, target_robot_positions[z_motor])
    check_out_of_limits(tx_motor, target_robot_positions[tx_motor])    

    # perform motions with respect to move_order
    for step in move_order:
        motors_to_move=[char_to_motor[ch] for ch in step] # determine which motors can move together each time
        # perform motion
        for m in motors_to_move:
            absolute_move(m, target_robot_positions[m])
        wait_for_motors(motors_to_move) # wait for step to complete
   


def close_gripper():
    absolute_move (gripper_motor, gripper_closed_pos)
    wait_for_motors([gripper_motor])

def open_gripper():
    absolute_move (gripper_motor, gripper_open_pos)
    wait_for_motors([gripper_motor])


if __name__=='__main__':
    # here it runs the main sequence to light the candles
    candles_z_pos=85
    shamash_x_pos=193
    candles_x_pos=[
        shamash_x_pos-84,
        shamash_x_pos-62,
        shamash_x_pos-44,
        shamash_x_pos-24,
        shamash_x_pos+24,
        shamash_x_pos+44,
        shamash_x_pos+67,
        shamash_x_pos+84]
    candles_y_tip=58+gripper_y_offset_mm
    shamash_y_tip=75+gripper_y_offset_mm

    # wait for user input to start
    while not Button.RIGHT in hub.buttons.pressed():
        wait(10)

    wait(10000) 
    st=StopWatch()
    st.reset()
    st.resume()

    #prepare for lighting
    absolute_move(z_motor, candles_z_pos-gripper_z_offset_mm)
    wait_for_motors([z_motor])
    wait(1500)
    close_gripper() #take candle from hand
    wait(500)
    absolute_move(tx_motor,45)
    wait_for_motors([tx_motor])
    wait(7000) # for user to light the candle

    #start going candle by candle and lighting them
    for candle_x in candles_x_pos:
        candle_tip_to(candle_x, candles_y_tip, candles_z_pos, 20, move_order=['yz','t','x'])
        wait(3000) #wait for candles to light up
    
    relative_move(tx_motor,-10)
    wait_for_motors([tx_motor])
    #go to the middle and place shamash
    candle_tip_to(shamash_x_pos, shamash_y_tip+25, candles_z_pos+7,0, move_order=['xy','t','z'])
    relative_move(tx_motor,-9)
    wait_for_motors([tx_motor])
    relative_move(z_motor,8)
    wait_for_motors([z_motor])
    wait(500)
    # lower shamash
    relative_move(y_motor,-22)
    wait_for_motors([y_motor])
    wait(500)
    open_gripper()
    relative_move(x_motor,2)
    wait_for_motors([x_motor])
    
    #return to start position
    go_home()
    print(f'elapsed: {st.time()/1000} sec')
    
