# tool.py

import numpy as np
import matplotlib.pyplot as plt
import re

alpha = 0.03

p1 = []
v1 = []
kp1 = []
kd1 = []
t1 = []

p2 = []
v2 = []
kp2 = []
kd2 = []
t2 = []

p3 = []
v3 = []
kp3 = []
kd3 = []
t3 = []

dt = 0.01
m = 0

theta1 = []
theta2 = []
theta3 = []
omega1 = []
omega2 = []
omega3 = []

def load_cur(cur_file):
    global n
    global theta1
    global theta2
    global theta3
    global omega1
    global omega2
    global omega3

    with open(cur_file, 'r') as data1_txt:
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

def load_ref(ref_file):
    global m
    m = 0
    with open(ref_file, 'r') as refs_txt:
        lns = refs_txt.readlines()
        for i in range(len(lns)):
            tmp = re.findall(r'-?\d+\.\d+', lns[i])
            if not (tmp == []):
                global p1
                global v1
                global kp1
                global kd1
                global t1
                global p2
                global v2
                global kp2
                global kd2
                global t2
                global p3
                global v3
                global kp3
                global kd3
                global t3

                p1 = p1 + [float(tmp[0])]
                v1 = v1 + [float(tmp[1])]
                kp1 = kp1 + [float(tmp[2])]
                kd1 = kd1 + [float(tmp[3])]
                t1 = t1 + [float(tmp[4])]
                p2 = p2 + [float(tmp[5])]
                v2 = v2 + [float(tmp[6])]
                kp2 = kp2 + [float(tmp[7])]
                kd2 = kd2 + [float(tmp[8])]
                t2 = t2 + [float(tmp[9])]
                p3 = p3 + [float(tmp[10])]
                v3 = v3 + [float(tmp[11])]
                kp3 = kp3 + [float(tmp[12])]
                kd3 = kd3 + [float(tmp[13])]
                t3 = t3 + [float(tmp[14])]
                m = m + 1

def write_ref_cpp(tbl_file, no):
    global p1
    global v1
    global kp1
    global kd1
    global t1
    global p2
    global v2
    global kp2
    global kd2
    global t2
    global p3
    global v3
    global kp3
    global kd3
    global t3

    with open(tbl_file, 'w+') as tbl_txt:
        tbl_txt.write("#include \"capstone.h\"\n")
        tbl_txt.write("#if usingRef == %s\n" % no)
        tbl_txt.write("struct refs refs_tbl[1000][3] = {\n")
        for i in range(m):
            tbl_txt.write("{ ")
            if 1 == 1:
                tbl_txt.write("{")
                tbl_txt.write(" .p_ref = %lf," % p1[i])
                tbl_txt.write(" .v_ref = %lf," % v1[i])
                tbl_txt.write(" .kp_ref = %lf," % kp1[i])
                tbl_txt.write(" .kd_ref = %lf," % kd1[i])
                tbl_txt.write(" .t_ref = %lf," % t1[i])
                tbl_txt.write(" }, ")
            if 2 == 2:
                tbl_txt.write("{")
                tbl_txt.write(" .p_ref = %lf," % p2[i])
                tbl_txt.write(" .v_ref = %lf," % v2[i])
                tbl_txt.write(" .kp_ref = %lf," % kp2[i])
                tbl_txt.write(" .kd_ref = %lf," % kd2[i])
                tbl_txt.write(" .t_ref = %lf," % t2[i])
                tbl_txt.write(" }, ")
            if 3 == 3:
                tbl_txt.write("{")
                tbl_txt.write(" .p_ref = %lf," % p3[i])
                tbl_txt.write(" .v_ref = %lf," % v3[i])
                tbl_txt.write(" .kp_ref = %lf," % kp3[i])
                tbl_txt.write(" .kd_ref = %lf," % kd3[i])
                tbl_txt.write(" .t_ref = %lf," % t3[i])
                tbl_txt.write(" }, ")
            tbl_txt.write("},\n")
        tbl_txt.write("};\n#endif\n")
        tbl_txt.flush()

def write_ref_tbl(tbl_file):
    global p1
    global v1
    global kp1
    global kd1
    global t1
    global p2
    global v2
    global kp2
    global kd2
    global t2
    global p3
    global v3
    global kp3
    global kd3
    global t3

    with open(tbl_file, 'w+') as tbl_txt:
        for i in range(m):
            if 1 == 1:
                tbl_txt.write("%lf\t" % p1[i])
                tbl_txt.write("%lf\t" % v1[i])
                tbl_txt.write("%lf\t" % kd1[i])
                tbl_txt.write("%lf\t" % kd1[i])
                tbl_txt.write("%lf\t" % t1[i])
            if 2 == 2:
                tbl_txt.write("%lf\t" % p2[i])
                tbl_txt.write("%lf\t" % v2[i])
                tbl_txt.write("%lf\t" % kd2[i])
                tbl_txt.write("%lf\t" % kd2[i])
                tbl_txt.write("%lf\t" % t2[i])
            if 3 == 3:
                tbl_txt.write("%lf\t" % p3[i])
                tbl_txt.write("%lf\t" % v3[i])
                tbl_txt.write("%lf\t" % kd3[i])
                tbl_txt.write("%lf\t" % kp3[i])
                tbl_txt.write("%lf\t" % t3[i])
            tbl_txt.write("\n")
        tbl_txt.write("\n")
        tbl_txt.flush()

def calc_next_ref():
    global p1
    global p2
    global p3
    global v1
    global v2
    global v3
    global kp1
    global kp2
    global kp3
    global kd1
    global kd2
    global kd3
    global t1
    global t2
    global t3
    global m

    def upd(ref, cur):
        return ref - alpha * (ref - cur)

    for i in range(m):
        p1[i] = upd(p1[i], theta1[i])
        v1[i] = upd(v1[i], omega1[i])
        kd1[i] = kp1[i]
        kd1[i] = kd1[i]
        t1[i] = t1[i]
        
        p2[i] = upd(p2[i], theta2[i])
        v2[i] = upd(v2[i], omega2[i])
        kd2[i] = kp2[i]
        kp2[i] = kd2[i]
        t2[i] = t2[i]
        
        p3[i] = upd(p3[i], theta3[i])
        v3[i] = upd(v3[i], omega3[i])
        kd3[i] = kp3[i]
        kd3[i] = kd3[i]
        t3[i] = t3[i]

def plot_err(val_name):
    global p1
    global p2
    global p3
    global v1
    global v2
    global v3
    global theta1
    global theta2
    global theta3
    global omega1
    global omega2
    global omega3
    global m

    theta1_err = []
    theta2_err = []
    theta3_err = []
    omega1_err = []
    omega2_err = []
    omega3_err = []

    t = np.linspace(0, dt * m, m, endpoint=True)

    for i in range(m):
        theta1_err = theta1_err + [p1[i] - theta1[i]]
        theta2_err = theta2_err + [p2[i] - theta2[i]]
        theta3_err = theta3_err + [p3[i] - theta3[i]]
        omega1_err = omega1_err + [v1[i] - omega1[i]]
        omega2_err = omega2_err + [v2[i] - omega2[i]]
        omega3_err = omega3_err + [v3[i] - omega3[i]]

    if val_name == 'theta1':
        val = theta1_err
    if val_name == 'theta2':
        val = theta2_err
    if val_name == 'theta3':
        val = theta3_err
    if val_name == 'omega1':
        val = omega1_err
    if val_name == 'omega2':
        val = omega2_err
    if val_name == 'omega3':
        val = omega3_err

    plt.plot(t,val,label=val_name+'_err')
    plt.legend()
    plt.show()

def plot_ref(val_name):
    global p1
    global p2
    global p3
    global v1
    global v2
    global v3
    global theta1
    global theta2
    global theta3
    global omega1
    global omega2
    global omega3
    global m

    t = np.linspace(0, dt * m, m, endpoint=True)

    if val_name == 'theta1':
        val = theta1
    if val_name == 'theta2':
        val = theta2
    if val_name == 'theta3':
        val = theta3
    if val_name == 'omega1':
        val = omega1
    if val_name == 'omega2':
        val = omega2
    if val_name == 'omega3':
        val = omega3

    plt.plot(t,val,label=val_name)
    plt.legend()
    plt.show()

def plot_cur(val_name):
    global p1
    global p2
    global p3
    global v1
    global v2
    global v3
    global theta1
    global theta2
    global theta3
    global omega1
    global omega2
    global omega3
    global m

    
    if val_name == 'theta1':
        val = theta1
    if val_name == 'theta2':
        val = theta2
    if val_name == 'theta3':
        val = theta3
    if val_name == 'omega1':
        val = omega1
    if val_name == 'omega2':
        val = omega2
    if val_name == 'omega3':
        val = omega3
    t = np.linspace(0, dt * len(val), len(val), endpoint=True)
    plt.plot(t,val,label=val_name)
    plt.legend()
    plt.show()

b = True

if __name__ == '__main__':
    for i in range(100):
        if b:
            cmd = input('cmd: ')
            if cmd == "read ref":
                file = input('dir: ')
                load_ref(file)
            if cmd == "read cur":
                file = input('dir: ')
                load_cur(file)
            if cmd == "calc next ref":
                calc_next_ref()
            if cmd == "write ref tbl":
                file = input("dir: ")
                write_ref_tbl(file)
            if cmd == "write ref cpp":
                file = input('dir: ')
                no = input('no: ')
                write_ref_cpp(file, no)
            if cmd == "plot error":
                val_name = input('value: ')
                plot_err(val_name)
            if cmd == "plot ref":
                val_name = input('value: ')
                plot_ref(val_name)
            if cmd == "plot cur":
                val_name = input('value: ')
                plot_cur(val_name)
            if cmd == "print ref":
                print("motor1:")
                print("  motor1.p:")
                print(p1)
                print("  motor1.v:")
                print(v1)
                print("  motor1.kp:")
                print(kp1)
                print("  motor1.kd:")
                print(kd1)
                print("  motor1.t:")
                print(t1)
                print("")
                print("motor2:")
                print("  motor2.p:")
                print(p2)
                print("  motor2.v:")
                print(v2)
                print("  motor2.kp:")
                print(kp2)
                print("  motor2.kd:")
                print(kd2)
                print("  motor2.t:")
                print(t2)
                print("motor3:")
                print("  motor3.p:")
                print(p3)
                print("  motor3.v:")
                print(v3)
                print("  motor3.kp:")
                print(kp3)
                print("  motor3.kd:")
                print(kd3)
                print("  motor3.t:")
                print(t3)
                print("end.")
            if cmd == "clear":
                m = 0
                theta1 = []
                theta2 = []
                theta3 = []
                omega1 = []
                omega2 = []
                omega3 = []
                p1 = []
                v1 = []
                kp1 = []
                kd1 = []
                t1 = []
                p2 = []
                v2 = []
                kp2 = []
                kd2 = []
                t2 = []
                p3 = []
                v3 = []
                kp3 = []
                kd3 = []
                t3 = []
            if cmd == "quit":
                b = False
