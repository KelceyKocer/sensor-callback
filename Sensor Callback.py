from pyparrot.Minidrone import Mambo
from pyparrot.Minidrone import MinidroneSensors
from datetime import datetime
import time
from ftplib import FTP
import tempfile

#mamboAddr = "e0:14:14:f8:3d:d1"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
#mambo = Mambo(mamboAddr, use_wifi=False)

#print("trying to connect")
#success = mambo.connect(num_retries=3)
#print("connected: %s" % success)


class PID:
    def __init__(self, pos_start, start_time, setpoints, p, i, d, allowed_errors, speed_limits, hold, delat_time):
        self.start_pos = pos_start
        self.start_time = start_time
        self.setpoints = setpoints
        self. current = [0,0,0]
        self.error = [0,0,0]
        self.timestamp = [0.0]
        self.allowed_error = allowed_errors
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.sum_error = [0,0,0]
        self.previous_error = [0,0,0]
        self.proportional = [0,0,0]
        self.integral = [0,0,0]
        self.derivative = [0,0,0]
        self.output = [0,0,0]
        self.speed_limit = speed_limits
        self.hold = hold
        self.success = [False,False,False]
        self.delay = delay_time
        self.path = "/tmp/UNSY_329_Mambo_Data" + str(datetime.now().strftime("%Y%m%d%H%M").format()) + ".csv"
    
    def set_current_position(self):
        if mambo.sensors.flying_state == "landed":
            self.current[0] = round(mambo.sensors.sensors._dict['DronePosition_posx'],2) - self.start_pos[0]
            self.current[1] = round(mambo.sensors.sensors._dict['DronePosition_posy'],2) - self.start_pos[1]
            self.current[2] = round(mambo.sensors.sensors._dict['DronePosition_posz'],2) * -1
            return True
        else:
            return False
    def debug_output(self):
        print("%0.5.02f,%0.5.02f,%0.5.02f,%0.5,02f,%0.5.02f,%0.5.02f,%0.5.02f,%0.5.02f,%0.5.02f,%0.5.02f,%s,%s,%s"%
              (self.timestamp,
               self.current[0], self.setpoints[0], self.output[0],
               self.current[1], self.setpoints[1], self.output[1],
               self.current[2], self.setpoints[2], self.output[2],
               self.success[0], self.success[1], self.success[2]))
    def next_cycle_prep(self):
        for i in range(3): self.previous_errors[i] = self.error[i]
        self.last_time = time.time()
            
    def is_stable(self):
        for i in range(3):
            if abs(self.error[i]) < self.allowed_error[i]:
                if self.hold[i] >= 0:
                    self.hold[i]-=1
                if self.hold[i] == 0: self.success[i] = True
                if self.success[0] and self.success[1] and self.success[2]:
                    print("Success on all controlled axes")
                    mambo.flat_trim()
                    mambo.smart_sleep(1)
                    return True
                else:
                    return False
            
    def override(self):
            pass
        
        
    def set_output_command(self):
            for i in range(3): self.error[i] = self.setpoint[i] - self.current[i]
            for i in range(3): self.proportional[i] = self.Kp[i] * self.error[i]
            for i in range(3): self.sum_error[i] += self.error[i]
            for i in range(3): self.integral[i] = self.sum_error[i] *self.Ki[i]
            for i in range(3): self.derivative[i] = self.previous_error[i] * self.Kd[i]
            
            output = [0,0,0]
            for i in range(3): self.output[i] = self.proportional[i] + self.integral[i] + self.derivative[i]
            for i in range(3): self.output[i] = min(max(self.output[i], self.speed_limit[i]*-1),spelfe.speed_limit[i])
            
            
            
    def PID_Controller(controller):
            if mambo.sensors.flying_state == "emergency":
                print("Out of order")
                return True
            controller.timestamp = round(time.time()-controller.start_time,3)
            if not controller.set_current_position():
                print("Mambo is %s. Aborting." % mambo.sensors.flying_state)
                return True
            
            controller.set_output_command()
            controller.override()
            controller.debug_output()
            
            if controller.is_stable():
                print("Controller reports stable steady state.")
                return True
            
            mambo.fly_direct(
                roll=controller.output[1],
                pitch=controller.output[0],
                yaw=0,
                vertical_movement=controller.output[2],
                duration=controller.delay)
            controller.next_cycle_prep()
            
            while not PID_Controller(controller):
                pass
            return True
def main():
    print("Trying to connect")
    mamboAddr = "e0:14:14:f8:3d:d1"
    mambo = Mambo(mamboAddr, use_wifi=False)
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)
    print(mambo.sensors.flying_state)
    if success:
        sensor.update = "ts,speed_x,speed_y,speed_z,posx,posy,posz,setpoint_x,setpoint_y,setpoint_z,output_x,output_y,output_z\n"
        takeoffMambo()
        mambo.smart_sleep(1)
        start_pos = [0,0,0]
        start_time = time.time()
        setpoints = [0,15,90]
        Kp = [0.2,0.2,0.1]
        Ki = [0.0,0.0,0.0]
        Kd = [0.0,0.0,0.0]
        allowed_errors = [5,5,5]
        speed_limits = [5,5,30]
        holds = [5,5,5]
        delay = 0.1
        
        print("timestamp,x_current,x_setpoint,x_output,y_current,y_setpoint,y_output,z_current,z_setpoint,z_output")
        controller = PID(start_pos, start_time, setpoints, Kp, Ki, Kd, allowed_errors, speed_limits, holds, delay)
        mambo.set_user_sensor_callback(updateMonitor,controller)
        myFile = open(controller.path, "a")
        myFile.write(sensor_update)
        myFile.close()
        
        print("Moving to: [0,15,90]")
        while not PID_Controller(controller):
            pass
        setpoints = [15,15,120]
        holds = [5,5,5]
        controller = PID(start_pos, start_time, setpoints, Kp, Ki, Kd, allowed_errors, speed_limits, holds, delay)
        mambo.set_user_sensor_callback(updateMonitor,controller)
        print("Moving to: [15,15,120]")
        while not PID_Controller(controller):
            pass
        
        setpoints = [15,0,90]
        holds = [5,5,5]
        controller = PID(start_pos, start_time, setpoints, Kp, Ki, Kd, allowed_errors, speed_limits, holds, delay)
        mambo.set_user_sensor_callback(updateMonitor,controller)
        print("Moving to: [15,0,90]")
        while not PID_Controller(controller):
            pass
        mambo.turn_degrees(90)
        mambo.smart_sleep(1)
        mamboLand()
    print("End")
    
            