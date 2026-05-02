from pymavlink import mavutil
import math

class Bridge(object):
    ##连接rov
    """
    Args:
        device (str, optional): Input device
            https://ardupilot.github.io/MAVProxy/html/getting_started/starting.html#master
        baudrate (int, optional): Baudrate for serial communication
    """
    def __init__(self,device = 'udp:192.168.2.1:14550',baudrate = 115200):
        self.master = mavutil.mavlink_connection(device,baudrate)
        self.master.wait_heartbeat()
        self.data = {}
    
    ##解锁rov
    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

    ##上锁rov
    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

    ##获取rov数据:dict
    def get_data(self):
        self.update()
        return self.data
    
    ##获取所有mavlink messages数据:dict
    def get_all_msgs(self):
        msgs = []
        while True:
            msg = self.master.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    ##更新rov data
    def update(self):
        msgs = self.get_all_msgs()
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    ##打印rov数据
    def print_data(self):
        self.update()
        print(self.data)

    ##设置rov模式
    def set_mode(self,mode):
        """ Set ROV mode
            http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MMAVLink mode
        {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 
        'GUIDED': 4, 'CIRCLE': 7, 'SURFACE': 9, 'POSHOLD': 16, 'MANUAL': 19}
        Returns:
            TYPE: Description
        """
        mode = mode.upper()
        #print(self.master.mode_mapping())
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            return
        mode_id = self.master.mode_mapping()[mode]
        #print("mode_id: ",mode_id)
        # self.master.set_mode(mode_id)         ##这个指令没用
        self.master.mav.set_mode_send(                                                      ##深度保持alt_hold验证有效         
        self.master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    
    ##手动控制rov
    def manual_control(self,x,y,z,r):
        #畸变系数，开环微调rov，尽量实现正前和正偏移行进
        # diff_x = 0.9
        # diff_y = 0.75

        # x = int(x * diff_x + y * math.sqrt(1-diff_y * diff_y))  #加上还是减去补偿值得推敲

        # y = int(y * diff_y + x * math.sqrt(1-diff_x * diff_x))

        #如果上述不行的话，人为观察提供补偿量
        # print("实际控制输入: x: ",x,"  y: ",y,"  z: ",z,"  yaw: ",r)

        x = max(-1000,min(x,1000))
        y = max(-1000,min(y,1000))
        z = max(0,min(z,1000))
        r = max(-1000,min(r,1000))

        buttons = 1 + 1 << 3 + 1 << 7
        self.master.mav.manual_control_send(
            self.master.target_system,
            ##x,y,z,r为油门大小
            ## x: range[-1000,1000]  >0：前进
            ## y: range[-1000,1000]  >0: 右侧横移
            ## z: range[  0,  1000]  >250:上浮
            ## r: range[-1000,1000]  >0: 右转
            x,    #[-1000,1000]          0-1000前进
            y,    #[-1000,1000]          0-1000右侧横移 
            z,  #[0,1000]                0-500下潜 
            r,    #[-1000,1000]          0-1000右转
            buttons)
    
    ##application function by KingO
    def go_circlre(self,rotation_radius,line_velocity,dirrection,cof):
        
        # cof = 0.45  #需要调试

        x = 0
        y = 0
        z = 500
        yaw = 0

        if dirrection == True:   #顺时针
            y = -line_velocity
            yaw = (int)((line_velocity/rotation_radius) * cof)
        
        else:                    #逆时针
            y = line_velocity
            r = (int)(-(line_velocity/rotation_radius) * cof)

        x = max(-1000,min(x,1000))
        y = max(-1000,min(y,1000))
        z = max(0,min(z,1000))
        yaw = max(-1000,min(r,1000))

        print("实际控制输入: x: ",x,"  y: ",y,"  z: ",z,"  yaw: ",r)

        self.manual_control(x,y,z,yaw)

    '''
    以下未测试
    '''

    ##从heartbeat中解码当前rov模式和arm状态
    def decode_mode(self,base_mode,custom_mode):
        """ Decode mode from heartbeat
            https://mavlink.io/en/messages/common#heartbeat

        Args:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        Returns:
            [str, bool]: Type mode string, arm state
        """
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm        

    ##获取rov当前模式以及是否解锁
    def get_mode(self):
        self.update()
        try:
            base_mode = self.get_data()['HEARTBEAT']['base_mode']
            custom_mode = self.get_data()['HEARTBEAT']['custom_mode']
            return self.decode_mode(base_mode, custom_mode)
        except:
            return 'MANUAL',False

    ##跟踪模式，与openMV配合使用
    def set_guided_mode(self):
        """ Set guided mode
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)
    
    ##发送命令信息，可参考self.arm()
    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ Function to abstract long commands

        Args:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.master.mav.command_long_send(
            self.master.target_system,                # target system
            self.master.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    ##设置位置目标,local_ned坐标系上
    def set_position_target_local_ned(self, param=[]):
        """ Create a SET_POSITION_TARGET_LOCAL_NED message
            https://mavlink.io/en/messages/common#SET_POSITION_TARGET_LOCAL_NED

        Args:
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        # Set mask
        mask = 0b0000000111111111
        for i, value in enumerate(param):
            if value is not None:
                mask -= 1<<i
            else:
                param[i] = 0.0

        #https://mavlink.io/en/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.master.mav.set_position_target_local_ned_send(
            0,                                              # system time in milliseconds
            self.master.target_system,                        # target system
            self.master.target_component,                     # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
            mask,                                           # mask: Bitmap to indicate which dimensions should be ignored by the vehicle
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate

    #设置姿态目标
    def set_attitude_target(self, param=[]):
        """ Create a SET_ATTITUDE_TARGET message
            https://mavlink.io/en/messages/common#SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.master.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.master.target_system,                # target system
            self.master.target_component,             # target component
            mask,                                   # mask: Bitmap to indicate which dimensions should be ignored by the vehicle
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust
    
    ##设置舵机转角
    def set_servo_pwm(self, id, pwm=1500):
        """ Set servo pwm

        Args:
            id (int): Servo id
            pwm (int, optional): pwm value 1100-2000
        """

        #https://mavlink.io/en/messages/common#MAV_CMD_DO_SET_SERVO
        # servo id
        # pwm 1000-2000
        mavutil.mavfile.set_servo(self.master, id, pwm)

    ##直接设置RC(已测试！！！！！！，没maunal_control好用，适合写底层)
    ##RC_id映射表见：http://www.ardusub.com/developers/rc-input-and-output.html
    def set_rc_channel_pwm(self, id, pwm=1100):#[1100,2000]
        """ Set RC channel pwm value

        Args:
            id (TYPE): Channel id
            pwm (int, optional): Channel pwm value 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id] = pwm
        #https://mavlink.io/en/messages/common#RC_CHANNELS_OVERRIDE
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.

    """
    以下是初步数据提取，待测试
    """
    ##标定压力传感器
    def calibrate_pressure(self):
        self.master.calibrate_pressure()
    
    ##获取压力传感器数据(已测试！！！！！！！！！！！！！！！)
    def get_pressure_data(self):
        self.update()
        try:
            bar30_data       = self.get_data()['SCALED_PRESSURE2']
            time_pressure_ms = bar30_data['time_boot_ms']
            press_abs        = bar30_data['press_abs']
            press_diff       = bar30_data['press_diff']
            temperature      = bar30_data['temperature'] / 100.0

            return time_pressure_ms,press_abs,press_diff,temperature
        except:
            return 0,0,0,0
    ##利用压力传感器转换深度
    def get_depth(self):
        FLUID_DENSITY = {'fresh':9.97,'salt':10.29}
        # Assume pressure_diff is temperature compensated
        # https://github.com/bluerobotics/ardusub/blob/978cd64a1e3b0cb5ba1f3bcc995fcc39bea7e9ff/libraries/AP_Baro/AP_Baro_MS5611.cpp#L481
        # https://github.com/bluerobotics/ms5837-python/blob/c83bdc969ea1654a2e2759783546245709bd9914/ms5837.py#L146
        time_pressure_ms,press_abs,press_diff,temperature = self.get_pressure_data()
        depth = press_diff / (FLUID_DENSITY['fresh'] * 9.80665)
        return depth * 100   ##cm为单位

    ##获取IMU数据(已测试，正确)
    #ATTITUDE
    def get_attitude(self):
        self.update()
        try:
            imu_data       = self.get_data()['ATTITUDE']
            time_imu_ms    = imu_data['time_boot_ms']
            roll           = imu_data['roll']
            pitch          = imu_data['pitch']
            yaw            = imu_data['yaw']
            rollspeed      = imu_data['rollspeed']
            pitchspeed     = imu_data['pitchspeed']
            yawspeed       = imu_data['yawspeed']

            return time_imu_ms,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed
        except:
            return 0,0,0,0,0,0,0

    ##获取电池电压和电流大小
    def get_battery(self):
        self.update()
        try:
            v = self.get_data()['SYS_STATUS']['voltage_battery'] / 1000.0
            a = self.get_data()['SYS_STATUS']['current_battery'] / 100.0
            return v,a
        except:
            return 0.0,0.0

    ##获取x,y,z,r设定速度pwm
    def get_cmd_vel(self):
        self.update()
        try:
            pwm_linear_z = self.get_data()['RC_CHANNELS']['chan3_raw']
            pwm_angular_z = self.get_data()['RC_CHANNELS']['chan4_raw']
            pwm_linear_x = self.get_data()['RC_CHANNELS']['chan5_raw']
            pwm_linear_y = self.get_data()['RC_CHANNELS']['chan6_raw']
            return pwm_linear_z,pwm_angular_z,pwm_linear_x,pwm_linear_y
        except:
            return 1500,1500,1500,1500