import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import math

def funcSecondOrder(x, a, b, c):
    return a + np.array(x) * b + c * np.square(x)

def funcLinear(x, a, b):
    return a + np.array(x) * b;

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

    # Find 
    poptX, pcovX = curve_fit(funcSecondOrder, tmp[0], tmp[1])
    poptY, pcovY = curve_fit(funcSecondOrder, tmp[0], tmp[2])
    print(poptX)
    print(poptY)

    xs = np.linspace(0., 0.4, 200)
    ys = []
    for x in xs:
        ys.append(funcSecondOrder([x], *poptX))

    ys2 = []
    for x in xs:
        ys2.append(funcSecondOrder([x], *poptY))

    plt.figure()
    plt.xlabel('bending angle', fontsize=18)
    plt.ylabel('x', fontsize=18)
    plt.scatter(tmp[0], tmp[1])
    plt.plot(xs, ys, 'r')
    plt.savefig('data/modelx.png')

    plt.figure()
    plt.xlabel('bending angle', fontsize=18)
    plt.ylabel('y', fontsize=18)
    plt.scatter(tmp[0], tmp[2])
    plt.plot(xs, ys2, 'r')
    plt.savefig('data/modely.png')

    plt.show()
