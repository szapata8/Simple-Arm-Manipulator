from transforms import *
import serial
import serial.tools.list_ports
import serial.serialutil
import time
from numpy import sin, cos
import pandas as pd
from scipy import signal

eye = np.eye(4)
pi = np.pi

STANDARD_SERIAL_WAIT = 0.1 # Seconds of wait before systems reads the serial buffer
BAUD_RATE = 9600
ESP32_ID = "PID=10C4:EA60"
INITIALIZE_HARDWARE_MSG = "INIT"
INITIALIZATION_MESSAGE_LENGTH = 40
COMPENSATION_DEGREES = [-6, -6, -3, -13, 12]

def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for i, port in enumerate(ports):
        # print(f"port #{i}: {port}. hwid: {port.hwid}. name: {port.name}. description: {port.description}. device: {port.device}")
        if ESP32_ID in port.hwid:
            return port.name
    print("ERROR: COM port for ESP32 not found")
    return None

class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts one link of dh parameters and returns a function "f" that will generate a
    homogeneous transform "A" given "q" as an input. A represents the transform from 
    link i to link i+1

    Parameters:
    dh - 1 x 4 list or iterable of floats, dh parameter table for one transform from link i to link i+1,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the transform from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            def A(q):
                theta = dh[0] + q
                d = dh[1]
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                return np.array([ [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                                  [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                                  [0, sin(alpha), cos(alpha), d],
                                  [0, 0, 0, 1]])


        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                theta = dh[0]
                d = dh[1] + q
                a = dh[2]
                alpha = dh[3]

                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of theta, d, a, and alpha variables as defined above. 

                return np.array([ [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                                  [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                                  [0, sin(alpha), cos(alpha), d],
                                  [0, 0, 0, 1]])


        self.A = A

class RobotArm:
    def __init__(self, dh:list, robot_setup_information=None, jt=None, base=eye, tip=eye, joint_limits=None):
        """
        arm = SerialArm(dh, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list or iterable of length 4 list or iterables representing dh parameters, [d theta a alpha]
        :param jt: n length list or iterable of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy or sympy array representing SE3 transform from world frame to frame 0
        :param tip: 4x4 numpy or sympy array representing SE3 transform from frame n to tool frame
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        self.dh = dh
        self.n = len(dh)
        self.com_port = find_esp32_port()
        self.microcontroller_serial_handle = None
        self.robot_setup_information = robot_setup_information

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # generating the function A(q) for each set of DH parameters
        for i in range(self.n):
            # TODO use the class definition above (dh2AFunc), and the dh parameters and joint type to
            # make a function and then append that function to the "transforms" list. 
            f = dh2AFunc(self.dh[i], self.jt[i])
            self.transforms.append(f.A)

        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.joint_limits = joint_limits
        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][0]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[0], dh[2]]))

        # Initialize serial connection
        self.initialize_serial_connection()

    def send_serial_instruction(self, instruction):
        print(f"send_serial_instruction. instruction: {instruction}")
        self.microcontroller_serial_handle.write(bytearray(instruction))
        time.sleep(STANDARD_SERIAL_WAIT) # Wait for the serial message to be received and stored in the buffer
        bytes_available = self.microcontroller_serial_handle.inWaiting()  # Read bytes available in the serial buffer
        hw_response = self.microcontroller_serial_handle.read(bytes_available)
        return [bytes_available, hw_response]

    def initialize_serial_connection(self):
        # Create instance of Serial
        self.microcontroller_serial_handle = serial.Serial(port = self.com_port, baudrate = BAUD_RATE) # Insert parameters for microcontroller connection
        while self.microcontroller_serial_handle.inWaiting() < INITIALIZATION_MESSAGE_LENGTH:
            continue
        time.sleep(0.5)
        bytesToRead = self.microcontroller_serial_handle.inWaiting() 
        response = self.microcontroller_serial_handle.read(bytesToRead)
        self.microcontroller_serial_handle.flushInput()
        self.microcontroller_serial_handle.flushOutput()
        print(f"ESP32 detected: {response}")
        # # Send command with robot setup information -> ability to initialize robot with variable amount of joints 
        # joint_amount = self.robot_setup_information["joint_amount"]
        # init_message = INITIALIZE_HARDWARE_MSG + f"{joint_amount}"
        # response = self.send_serial_instruction(init_message)

        # print(f"Response to initialization: {response}")

        return response

    def update_hardware_q(self, q, verbose=False):

        print(f"q command received: {q}")
        if self.microcontroller_serial_handle == None:
            print("ERROR: Serial handle not available. Initialize serial connection first.")
            return False
        
        # Extract first command and separate it into 2 different angle commands (base joint covers 180 and 180 with 2 separate servos to cover 360 degrees)
        q_updated = [int(q[0]/2), int(q[0]/2)]
        for index in range(1,len(q)):
            q_updated.append(int(q[index])) # Skip the first term since it's already been included
        
        # Compensate to get to 0
        for i in range(len(q_updated)):
            q_updated[i] += COMPENSATION_DEGREES[i]
            if q_updated[i] < 0:
                q_updated[i] = 0

        print(f"q_updated: {q_updated}")
        hw_response = self.send_serial_instruction(q_updated)

        # Print the response of the hardware
        if verbose:
            print(f"Hardware Response: \n{hw_response}")
        
        # return the response
        return hw_response

    def fk(self, q, index=None, base=False, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index - integer or list of two integers. If a list of two integers, the first integer represents the starting JOINT 
                    (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                    the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        # the following lines of code are data type and error checking. You don't need to understand
        # all of it, but it is helpful to keep. 

        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None

        # TODO complete each of the different cases below. If you don't like the 
        # current setup (in terms of if/else statements) you can do your own thing.
        # But the functionality should be the same. 
        if base and start_frame == 0:
            T = self.base
        else:
            T = eye

        for i in range(start_frame, end_frame):
            T = T @ self.transforms[i](q[i])

        if tip and end_frame == self.n:
            T = self.tip

        return T

    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the end effector frame of the arm in a given configuration

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """


        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        # TODO - start by declaring the correct size for the Jacobian
        J = np.zeros([6, self.n])

        # TODO - find the current position of the gripper "pe" using your fk function
        # this will likely require additional intermediate variables than what is shown here.
        end_effector_frame = self.fk(q, base=base, tip=tip) 
        pe = end_effector_frame[0:3,3] # Grab the 4th column of the homogeneous transform (position)
        # print(f"pe: {pe}")


        # TODO - calculate all the necessary values using your "fk" function, and fill every column
        # of the jacobian using this "for" loop. Functions like "np.cross" may also be useful. 
        for i in range(index):
            i_minus_1_frame = self.fk(q, i) # It's i not i-1 since python starts at 0
            z_i_minus_1 = i_minus_1_frame[0:3, 2] # Grab the Z axis definition of the frame
            # check if joint is revolute
            if self.jt[i] == 'r':
                p_i_minus_1 = i_minus_1_frame[0:3, 3] # Grab the position of the current frame
                p_vector = pe - p_i_minus_1  # Grab the position of the end effector with respect to the current frame
                J[0:3, i] = np.cross(z_i_minus_1, p_vector)
                J[3:6, i] = z_i_minus_1

            # if not assume joint is prismatic
            else:
                # Add more stuff here
                J[0:3, i] = z_i_minus_1

        return J
        
    def compute_robot_path(self, q_init, goal, obst_location, obst_radius, joint_limits):
        print(f"q_init: {q_init}")
        print(f"goal: {goal}")
        print(f"obst_location: {obst_location}")
        print(f"obst_radius: {obst_radius}")
        print(f"joint_limits: {joint_limits}")
        # some initialization:
        q_s = []
        error = None
        count = 0
        maximum_reach = 0

        # Import constants:
        max_iter=1000
        tol=1e-4
        K=np.eye(3)
        kd=0.0001
        dt=0.01

        # Perform some checks:
        if isinstance(q_init, np.ndarray):
                    q = q_init
        elif q_init == None:
                q = np.zeros(1,4)
        else:
                q = np.array(q_init)

        for i in range(self.n):  # Add max length of each link
                maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = goal  
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2) # Find distance to target

        if target_distance > maximum_reach:
                print("WARNING: Target outside of reachable workspace!")
                return q, error, count, False, "Failed: Out of workspace"
        else:
                if target_distance > maximum_reach:
                    print("Target out of workspace, but finding closest solution anyway")
                else:
                    print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
                    float(target_distance), float(maximum_reach)))

        # Some functions for cleanliness:
        def get_error(q, point, index=None):
                cur_position = self.fk(q=q, index=index)[0:3,3]
                e = point-cur_position
                return e

        def midpoint_error(q, point, index1=[0,0], index2=None):
                joint1 = self.fk(q=q, index=index1)
                joint2 = self.fk(q=q, index=index2)
                midpoint = (joint1[0:3,3] + joint2[0:3,3])/2
                e = point - midpoint
                # print('join1', joint1)
                # print('joint2', joint2)
                # print('midpoint', midpoint)
                # print('e', e)
                return e

        def get_fr(nu=0.2, envelope=0.2, kr=0.2, er_r=[]):
                if nu < envelope:
                    fr = (kr/nu**4) * np.gradient(er_r)
                else:
                    fr = er_r*0.0
                return fr

        # Find initial errors:
        er_a = get_error(q,pt) # distance from tip to goal
        er_r1 = get_error(q,obst_location) # distance from tip to object
        er_r2 = get_error(q,obst_location,index=[0,3]) # distance from last revolute joint to object
        er_r3 = midpoint_error(q, obst_location, index1=[0,2], index2=[0,3]) # distance from midpoint on 2nd link to object
        er_r4 = get_error(q, obst_location, index=[0,2]) # distance from first revolute joint to object
        er_r5 = get_error(q, [0, 0, 0]) # distance from tip to base1
        er_r6 = get_error(q, [0, 0, 0.1]) # distance from tip to base2

        # Generate q_s using Artificial Potential Field method:
        while np.linalg.norm(er_a) > tol and count < max_iter: 
                J = self.jacob(q)[0:3,:]
                
                # Generate Attractive Field
                ka = 0.5
                fa = ka * er_a

                # Generate Repulsive Field for object
                nu1 = np.linalg.norm(er_r1) - obst_radius
                nu2 = np.linalg.norm(er_r2) - obst_radius
                nu3 = np.linalg.norm(er_r3) - obst_radius
                nu4 = np.linalg.norm(er_r4) - obst_radius
                nu5 = np.linalg.norm(er_r5) - obst_radius
                nu6 = np.linalg.norm(er_r6) - obst_radius


                if self.fk(q_init)[2,3] < goal[2]:
                    fr1 = get_fr(nu=nu1, envelope=0.3, kr=0.6, er_r=er_r1)
                    fr2 = get_fr(nu=nu2, envelope=0.2, kr=0.6, er_r=er_r2)
                    fr3 = get_fr(nu=nu3, envelope=0.2, kr=0.3, er_r=er_r3)
                    fr4 = get_fr(nu=nu4, envelope=0.2, kr=0.6, er_r=er_r4)
                    fr5 = get_fr(nu=nu5, envelope=0.2, kr=0.6, er_r=er_r5)
                    fr6 = get_fr(nu=nu6, envelope=0.2, kr=0.6, er_r=er_r6)

                else:
                    fr1 = get_fr(nu=nu1, envelope=0.3, kr=0.6, er_r=er_r1)
                    fr2 = get_fr(nu=nu2, envelope=0.3, kr=0.6, er_r=er_r2)
                    fr3 = get_fr(nu=nu3, envelope=0.4, kr=0.3, er_r=er_r3)
                    fr4 = get_fr(nu=nu4, envelope=0.2, kr=0.6, er_r=er_r4)
                    fr5 = get_fr(nu=nu5, envelope=0.2, kr=0.6, er_r=er_r5)
                    fr6 = get_fr(nu=nu6, envelope=0.2, kr=0.6, er_r=er_r6)
                
                # Calculate next q
                error = fa + fr1 + fr2 + fr3 + fr4 # + fr5 + fr6  # sum of attractive and repulsive fields
                if (self.fk(q=q)[2,3] + error[2]) < 0: # if the EE is about to go under the floor,
                    error[2] = 0 # set the z-velocity equal to zero
                Ker = K @ error
                qdot = J.T @ np.linalg.inv((J@J.T) + (kd**2*np.eye(3))) @ Ker  # use 'pinv' method
                q = q + qdot*dt

                # make sure q stays within joint limits
                for i in range(len(q)):
                    if q[i] <= (joint_limits[i][0]+5):
                            q[i] = joint_limits[i][0]+5
                    elif q[i] >= (joint_limits[i][1]-5):
                            q[i] = joint_limits[i][1]-5
                
                # Update errors
                er_a = get_error(q,pt)
                er_r1 = get_error(q,obst_location)
                er_r2 = get_error(q,obst_location,index=[0,3])
                er_r3 = midpoint_error(q, obst_location, index1=[0,2], index2=[0,3])
                er_r4 = get_error(q, obst_location, index=[0,2])
                er_r5 = get_error(q, [0, 0, 0]) # distance from tip to base1
                er_r6 = get_error(q, [0, 0, 0.1]) # distance from tip to base2

                count = count+1
                q_s.append(q)
                
        q_og = pd.DataFrame(q_s)
        q_smooth = np.zeros([1000,4])
        for i in range(len(q_smooth.T)):
                q_smooth[:,i] = signal.savgol_filter(q_og[i], 501, 6)
        q_out = np.array(q_smooth)
        
        return q_out     
            

def testing():
    # Create a test function that connects to the hardware and is able to receive input form user over the console to send it to the robot
    pass

if __name__ == "__main__":
    testing()