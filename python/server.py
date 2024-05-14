#!/usr/bin/python3

# COPYRIGHT HERE!

# IMPORT -> Required libraries:
import rclpy
from rclpy.node import Node
from ros2_robotiqgripper.srv import RobotiqGripper
import socket, time, re

# ===== INPUT PARAMETER ===== #
PARAM_IP = "0.0.0.0"
P_CHECK_IP = False

class ipPARAM(Node):
    
    def __init__(self):

        global PARAM_IP
        global P_CHECK_IP
        
        super().__init__('ros2_robotiq_ip_param')
        self.declare_parameter('IPAddress', "None")

        PARAM_IP = self.get_parameter('IPAddress').get_parameter_value().string_value
        
        if (PARAM_IP == "None"):

            print('IPAddress ROS2 Parameter was not defined for the ros2_robotiq Service Server.')
            exit()

        else:    
            print('IPAddress ROS2 Parameter received: ' + PARAM_IP)

        P_CHECK_IP = True

# Create NODE:
class serviceServer(Node):

    def __init__(self, IP):

        # Initialise ROS 2 Service Server:
        super().__init__('ros2_RobotiqGripper_ServiceServer')
        self.SERVICE = self.create_service(RobotiqGripper, "Robotiq_Gripper", self.ExecuteService)

        self.ip = IP

    def ExecuteService(self, request, response):

        # INITIALISE RESPONSE:
        response.success = False
        response.value = -1
        response.average = -1.0
        
        # TCP-IP + SOCKET settings:
        HOST = self.ip
        PORT = 63352
        
        # SOCKET COMMUNICATION:
        SCKT = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SCKT.settimeout(3) # Timeout of 3 seconds.

        # OPEN socket:
        while True:
            try:
                SCKT.connect((HOST, PORT))
                break
            except TimeoutError:
                response.message = "ERROR: TCP-IP socket connection timed out! Please verify IP address and PORT."
                return(response)
            except ConnectionRefusedError:
                response.message = "ERROR: TCP-IP socket connection was refused! Please verify IP address and PORT."
                return(response)

        if request.action == "CLOSE":
            
            SCKT.sendall(b'SET POS 255\n')
            ignore = SCKT.recv(2**10)
            time.sleep(1.0)
            SCKT.sendall(b'GET POS\n')
            data = SCKT.recv(2**10)

            GripperPos_STR = int(re.search(r'\d+', str(data)).group())
            AVERAGE = round((float(GripperPos_STR)/255.0)*100.0, 2)
            
            response.success = True
            response.value = GripperPos_STR
            response.average = AVERAGE
            response.message = "CLOSE command successfully sent to Robotiq gripper. After execution, the gripper is -> " + str(AVERAGE) + "% CLOSED."
            return(response)

        elif request.action == "OPEN":
            
            SCKT.sendall(b'SET POS 0\n')
            ignore = SCKT.recv(2**10)
            time.sleep(1.0)
            SCKT.sendall(b'GET POS\n')
            data = SCKT.recv(2**10)

            GripperPos_STR = int(re.search(r'\d+', str(data)).group())
            AVERAGE = round((float(GripperPos_STR)/255.0)*100.0, 2)
            
            response.success = True
            response.value = GripperPos_STR
            response.average = AVERAGE
            response.message = "OPEN command successfully sent to Robotiq gripper. After execution, the gripper is -> " + str(AVERAGE) + "% CLOSED."
            return(response)

        else:
            response.message = "ERROR: Valid commands are OPEN/CLOSE. Please try again."
            return(response)

# =================== MAIN =================== #
def main(args=None):

    rclpy.init(args=args)

    # Get IP Address:PROGRAM
    global PARAM_IP
    global P_CHECK_IP

    paramNODE = ipPARAM()
    while (P_CHECK_IP == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()
    
    # Initialise NODE:
    GripperNode = serviceServer(PARAM_IP)
    print ("[ROS2 Robotiq Gripper]: ros2_RobotiqGripper_ServiceServer generated.")

    # Spin SERVICE:
    rclpy.spin(GripperNode)
    
    GripperNode.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()


