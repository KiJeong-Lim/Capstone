# recover.py

import re

theta1 = []
theta2 = []
theta3 = []
omega1 = []
omega2 = []
omega3 = []

with open('c0.txt', 'r') as data1_txt:
    lns = data1_txt.readlines()
    for i in range(len(lns)):
        tmp = re.findall(r'-?\d+\.\d+', lns[i])
        if i % 4 == 3:
            pass
        else:
            theta = tmp[0]
            omega = tmp[1]
            if i % 4 == 0:
                theta1 = theta1 + [float(theta)]
                omega1 = omega1 + [float(omega)]
            if i % 4 == 1:
                theta2 = theta2 + [float(theta)]
                omega2 = omega2 + [float(omega)]
            if i % 4 == 2:
                theta3 = theta3 + [float(theta)]
                omega3 = omega3 + [float(omega)]

with open('c0', 'w+') as new_txt:
    for i in range(1000):
        new_txt.write("theta1: %lf, " % theta1[i])
        new_txt.write("omega1: %lf\n" % (omega1[i] / 100))
        new_txt.write("theta2: %lf, " % theta2[i])
        new_txt.write("omega2: %lf\n" % (omega2[i] / 100))
        new_txt.write("theta3: %lf, " % theta3[i])
        new_txt.write("omega3: %lf\n" % (omega3[i] / 100))
        new_txt.write("\n")
