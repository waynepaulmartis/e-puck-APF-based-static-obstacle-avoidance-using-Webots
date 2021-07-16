from controller import Robot, Motor, Supervisor, Node
import math
import numpy
import matplotlib.pyplot as plt
import pandas as pd


# time in [ms] of a simulation step
TIME_STEP = 64
v = 0.05 #linear velocity
r = 0.0205 #wheel radius
d = 0.052 #robot diameter
MAX_SPEED = 6.28
n_obs = 5

x = [-0.601, 1.0, 1.08, -0.35, -1.45] #obstacle coordinates
z = [0.952, -0.885, 1.27, -0.51, -0.93]

def mag(Xi, Zi, Xf, Zf):
    d = math.sqrt((Xi - Xf)**2 + (Zi - Zf)**2)
    return d

robot = Supervisor()

pos = robot.getFromDef("coordinate")
trans_val = pos.getField("translation")
rot_val = pos.getField("rotation")

x_coord = []
z_coord = []
x_g_rep_pf = []
z_g_rep_pf = []
x_g_at_pf = []
z_g_at_pf = []
x_pf = []
z_pf = []
time = []
heading_angle = []
omega = []
l_speed = []
r_speed = []

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

class epuc:

    def __init__(self, x_f, z_f): #initialize goal coordinates
        self.x_f = x_f
        self.z_f = z_f

    def set_params(self, x_i, z_i, Ka, Kr, xGradRepPotFieldTemp, zGradRepPotFieldTemp):
        self.x_i = x_i
        self.z_i = z_i
        self.Ka = Ka
        self.Kr = Kr
        self.xGradRepPotFieldTemp = xGradRepPotFieldTemp
        self.zGradRepPotFieldTemp = zGradRepPotFieldTemp

    def heading_angle_val(self, field_angle_temp, xGradPotField): #calculates heading angle in robot coordinate space
        if xGradPotField >= 0:
            self.heading_angle = field_angle_temp - 3.14
        else :
            self.heading_angle = field_angle_temp + 3.14

    def speed(self, r=0.0205, d=0.0502, v=0.05): #calculates wheel speed
        self.l = (2*v - d*self.w)/(2*r)
        self.r = (2*v + d*self.w)/(2*r)

    def rep_pf(self, x_pos, z_pos): #Khatib's repulsive potential field
        self.x_pos = x_pos
        self.z_pos = z_pos
        self.DistGoal = mag(self.x_pos,self.z_pos,self.x_f,self.z_f)
        for i in range(n_obs):
            DistObs = mag(self.x_pos,self.z_pos,x[i],z[i])
            do = 1.0 #influence range of obstacle
            if DistObs <= do:
                Gx1 = (self.Kr*((1/DistObs)-(1/do))*(1/DistObs**3)*(self.DistGoal)*(self.x_pos - x[i]))
                Gx2 = (0.5*self.Kr*(((1/DistObs)-(1/do))**2)*(1/self.DistGoal)*(self.x_pos - self.x_f))
                Gz1 = (self.Kr*((1/DistObs)-(1/do))*(1/DistObs**3)*(self.DistGoal)*(self.z_pos - z[i]))
                Gz2 = (0.5*self.Kr*(((1/DistObs)-(1/do))**2)*(1/self.DistGoal)*(self.z_pos - self.z_f))
                self.xGradRepPotFieldTemp = (Gx1-Gx2) + self.xGradRepPotFieldTemp
                self.zGradRepPotFieldTemp = (Gz1-Gz2) + self.zGradRepPotFieldTemp
                RepPotFieldTemp = self.Kr*(1/DistObs) 

    def att_pf(self):
        self.xGradAttPotField = -self.Ka*2*(self.DistGoal)*(self.x_pos - self.x_f)
        self.zGradAttPotField = -self.Ka*2*(self.DistGoal)*(self.z_pos - self.z_f)
        AttPotField = 1.0*self.DistGoal**2
    
    def angular_vel(self):
        w_temp = (self.heading_angle - a)
        if w_temp <= 2.0 and w_temp >= -2.0 :
            self.w = 1.5*w_temp
        elif w_temp <= 3.14 and w_temp >= -3.14 :
            self.w = 0.9*w_temp
        else :
            self.w = 0.45*w_temp

    def plot(self):#plotting
        plt.plot(z_coord, x_coord)
        plt.scatter(z_coord[0], x_coord[0], marker='o', color='green')
        plt.scatter(z_coord[k-2], x_coord[k-2], marker='o', color='red')
        plt.xlabel("Z axis")
        plt.ylabel("X axis")
        plt.title("Arena (-1.0,-0.5) to (0.5,-0.5)")
        plt.axis([-2, 2, -2, 2])
        rectangle1=plt.Rectangle((z[0]-0.126,x[0]-0.126), 0.252, 0.252, fc='r')
        rectangle2=plt.Rectangle((z[1]-0.126,x[1]-0.126), 0.252, 0.252, fc='r')
        rectangle3=plt.Rectangle((z[4]-0.126,x[4]-0.126), 0.252, 0.252, fc='r')
        plt.gca().add_patch(rectangle1)
        plt.gca().add_patch(rectangle2)
        plt.gca().add_patch(rectangle3)
        circle1 = plt.Circle((z[2], x[2]), radius=0.126, fc='y')
        circle2 = plt.Circle((z[3], x[3]), radius=0.126, fc='y')
        plt.gca().add_patch(circle1)
        plt.gca().add_patch(circle2)
        plt.grid(True)
        plt.show()

    def print_vals(self, field_angle, xGradPotField, zGradPotField):
        print("field angle:", field_angle)
        print("omega:", self.w)
        print("Att Pot Field X:", self.xGradAttPotField)
        print("Att Pot Field Z:", self.zGradAttPotField)
        print("Rep Pot Field X:", self.xGradRepPotFieldTemp)
        print("Rep Pot Field Z:", self.zGradRepPotFieldTemp)
        print("Pot Field X:", xGradPotField)
        print("Pot Field Z:", zGradPotField)

while robot.step(TIME_STEP) != -1:
	
    bot = epuc(1.5, -0.9)
    bot.set_params(-1.8, -0.9, 5.0, 0.5, 0.0, 0.0) #(Xi, Zi, Ka, Kr, xGradpf, zGradpf)

    values = trans_val.getSFVec3f() #retrieves position coordinates
    x_pos = values[0]
    y_pos = values[1]
    z_pos = values[2]
    x_coord.append(x_pos)
    z_coord.append(z_pos)
    t = robot.getTime()
    time.append(t)
    # print(z_pos)
    angles = rot_val.getSFRotation() #orientation angle
    a = round(angles[3],4)
    # print(a)

    bot.rep_pf(x_pos, z_pos)
    bot.att_pf()

    xGradPotField = bot.xGradRepPotFieldTemp + bot.xGradAttPotField
    zGradPotField = bot.zGradRepPotFieldTemp + bot.zGradAttPotField

    field_angle_temp = math.atan2(xGradPotField, zGradPotField)
    # field_angle = robot_coordinate(field_angle_temp,x_pos,z_pos)
    bot.heading_angle_val(field_angle_temp, xGradPotField)
    # field_angle = field_angle_temp + 3.14

    bot.angular_vel()
    bot.speed()

    x_g_rep_pf.append(bot.xGradRepPotFieldTemp)
    z_g_rep_pf.append(bot.zGradRepPotFieldTemp)
    x_g_at_pf.append(bot.xGradAttPotField)
    z_g_at_pf.append(bot.zGradAttPotField)
    x_pf.append(xGradPotField)
    z_pf.append(zGradPotField)
    heading_angle.append(bot.heading_angle)
    omega.append(bot.w)

    leftSpeed = bot.l
    rightSpeed = bot.r

    l_speed.append(leftSpeed)
    r_speed.append(rightSpeed)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    if mag(x_pos,z_pos,bot.x_f,bot.z_f) < 0.01:
        break

x_coordinate = numpy.array(x_coord)
z_coordinate = numpy.array(z_coord)
dct = {'x-axis' : [x_coord],
       'z-axis' : [z_coord]
       }
df1 = pd.DataFrame({'time' : time,
                    'x-axis' : x_coord,
                    'z-axis' : z_coord,
                    'x-rep-field' : x_g_rep_pf,
                    'z-rep-field' : z_g_rep_pf,
                    'x-att-field' : x_g_at_pf,
                    'z-att-field' : z_g_at_pf,
                    'x-field' : x_pf,
                    'z-field' : z_pf,
                    'heading angle' : heading_angle,
                    'omega' : omega,
                    'left speed' : l_speed,
                    'right speed' : r_speed})
df1.to_excel("trial21.xlsx", sheet_name="x coord")
    
bot.plot()

