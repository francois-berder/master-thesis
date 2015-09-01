import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import math

def funcQuadratic(x, a, b, c):
    return a + np.array(x) * b + c * np.square(x)

def funcLinear(x, a, b):
    return a + np.array(x) * b;

def findModel(name, f, datat, datax, datay):
    poptX, pcovX = curve_fit(f, datat, datax)
    poptY, pcovY = curve_fit(f, datat, datay)

    print(name + ' model:\n')
    print(poptX)
    print(poptY)
    print('\n')

    xs = np.linspace(0., 0.4, 200)
    ys = []
    for x in xs:
        ys.append(f([x], *poptX))

    ys2 = []
    for x in xs:
        ys2.append(f([x], *poptY))

    plt.figure()
    plt.xlabel('bending angle', fontsize=18)
    plt.ylabel('x', fontsize=18)
    plt.scatter(datat, datax)
    plt.plot(xs, ys, 'r')
    plt.savefig('model-' + name + '-x.png')
    plt.savefig('report/images/model-' + name + '-x.png')

    plt.figure()
    plt.xlabel('bending angle', fontsize=18)
    plt.ylabel('y', fontsize=18)
    plt.scatter(datat, datay)
    plt.plot(xs, ys2, 'r')
    plt.savefig('model-' + name + '-y.png')
    plt.savefig('report/images/model-' + name + '-y.png')

if __name__ == '__main__':

    # Read data
    modelFile = open("data/modeldata", "r")
    reader = csv.reader(modelFile, delimiter=' ')
    data = []
    for row in reader:
        data.append((float(row[0]), -float(row[1]), float(row[2])))


    # Normalizing vector
    for i, entry in enumerate(data):
        l = math.sqrt(entry[2] * entry[2] + entry[1] * entry[1])
        data[i] = (entry[0], entry[1]/l, entry[2]/l)

    tmp = list(zip(*data))

    findModel("linear", funcLinear, tmp[0], tmp[1], tmp[2])
    findModel("quadratic", funcQuadratic, tmp[0], tmp[1], tmp[2])

