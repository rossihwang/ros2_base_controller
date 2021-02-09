# Copyright <2020> [Copyright rossihwang@gmail.com]

import parse
import sys
import numpy as np
import matplotlib.pyplot as plt

'''
sample

[INFO] [1605797184.361561764] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.461661997] [base_controller_node]: DEBUG: led toggle
[INFO] [1605797184.491598099] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.561586024] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.661619667] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.761681337] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.861677281] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797184.961548388] [base_controller_node]: DEBUG: led toggle
[INFO] [1605797184.991613606] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797185.061628578] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797185.161592073] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797185.261609221] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
[INFO] [1605797185.361642342] [base_controller_node]: rtarget: 0, right: 0, ltarget: 0, left: 0
'''

def main():
    rtarget = np.array([])
    right = np.array([])
    ltarget = np.array([])
    left = np.array([])
    
    with open(sys.argv[1]) as f:
        for l in f:
            ret = parse.parse("{}rtarget: {}, right: {}, ltarget: {}, left: {}", l)
            if ret != None:
                rtarget = np.append(rtarget, float(ret[1]))
                right = np.append(right, float(ret[2]))
                ltarget = np.append(ltarget, float(ret[3]))
                left = np.append(left, float(ret[4]))

    plt.plot(rtarget)
    plt.plot(right)
    plt.plot(ltarget)
    plt.plot(left)
    plt.legend(["rtarget", "right", "ltarget", "left"])

    plt.show()    

if __name__ == "__main__":
    main()