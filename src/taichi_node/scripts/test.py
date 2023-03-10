import numpy as np
import vectormath as vm

def main():
    x = []
    x.append(0.1)
    x.append(0.5)
    x = 5 * [0.1, 0.2]

    y = vm.Vector3(0.1, 0.2, 100)
    y = 5 * y
    print(y)
    print(x)

if __name__=="__main__":
    main()