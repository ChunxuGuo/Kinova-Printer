import numpy
f = open('UMS5_50_50_100.gcode')
x = 0
y = 0
z = 0
path = []

lines = f.readlines()
for line in lines:
    if 'G0' in line or 'G1' in line:
        for cmd in line.split():
            if 'X' in cmd:
                x = cmd[1:]
            elif 'Y' in cmd:
                y = cmd[1:]
            elif 'Z' in cmd:
                z = cmd[1:]
        path.append([float(x)/1000,float(y)/1000,float(z)/1000])
        # print(line)
# print(path)
numpy.save('path.npy',path)
a = numpy.load('path.npy')

for i in range(200):
    print(a[i+10000])
