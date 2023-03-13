import numpy as np
import vectormath as vm
from fabrik.fabrikSolver import FabrikSolver2D, FabrikSolver3D

def main():
    # x = []
    # x.append(0.1)
    # x.append(0.5)
    # x = 5 * [0.1, 0.2]

    # y = vm.Vector3(0.1, 0.2, 100)
    # y = 5 * y
    # print(y)
    # print(x)
    arm = FabrikSolver3D()

    arm.addSegment(100, 200, 100.0)
    arm.addSegment(100, -20, 200)
    arm.addSegment(100, -20, 0)
    arm.addSegment(100, 20, 0)

    # arm.plot()

    for segment in arm.segments:
        print(segment.point)
    arm.compute(100, 150, 50)
    for segment in arm.segments:
        print(segment.point)

if __name__=="__main__":
    # main()
    x = [0.0]
    y = 6*x
    print(y)