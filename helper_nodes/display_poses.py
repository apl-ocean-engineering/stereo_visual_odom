import matplotlib.pyplot as plt
import copy
import numpy as np

import argparse


def get_eucld_error(x1, x2, y1, y2, z1, z2):
    error = []
    for i in range(len(x1)):
        p1 = np.array([x1[i], y1[i], z1[i]])
        p2 = np.array([x2[i], y2[i], z2[i]])

        error.append(float(np.sum(np.subtract(p1, p2))))

    return error


def get_data(f, ignore_count=[]):
    count = 0
    idx = []
    time = []
    x = []
    y = []
    z = []
    roll = []
    pitch = []
    yaw = []
    for line in f:
        idx.append(count)
        count += 1
        time.append(line.split(',')[0])
        x.append(float(line.split(',')[1]))
        y.append(float(line.split(',')[2]))
        z.append(float(line.split(',')[3]))
        roll.append(float(line.split(',')[4]))
        pitch.append(float(line.split(',')[5]))
        yaw.append(float(line.split(',')[6]))

    return idx, time, x, y, z, roll, pitch, yaw


def check_val(l1, l2, idx, ind, max_val, min_val):
    if l1[ind] > max_val or l2[ind] > max_val:
        l1.pop(ind)
        l2.pop(ind)
        idx.pop(ind)
    elif l1[ind] < min_val or l2[ind] < min_val:
        l1.pop(ind)
        l2.pop(ind)
        idx.pop(ind)
    else:
        ind += 1

    return l1, l2, idx, ind


def check_error_val(error, idx, ind, max_val, min_val):
    if error[ind] > max_val:
        error.pop(ind)
        idx.pop(ind)
    elif error[ind] < min_val:
        error.pop(ind)
        idx.pop(ind)
    else:
        ind += 1

    return error, idx, ind


def plot(idx_list, v1, v2, v3=None, title=" ",
         label1="1", label2="2", label3="3", dump=False, y_min=None,
         y_max=None):
    fig1, ax1 = plt.subplots()
    ax1.scatter(idx_list, v1, c='k', label=label1)
    ax1.scatter(idx_list, v2, c='g', label=label2)
    if v3 is not None:
        ax1.scatter(idx_list, v3, c='b', label=label3)
    ax1.set_title(title)
    if y_min is not None and y_max is not None:
        ax1.set_ylim((y_min, y_max))
    ax1.legend()
    if dump:
        fig1.savefig(title + ".png")


def plot_error(idx_list, error, title=" ",
               dump=False, y_min=None, y_max=None):
    fig1, ax1 = plt.subplots()
    ax1.scatter(idx_list, error, c='k')
    ax1.set_title(title)
    if y_min is not None and y_max is not None:
        ax1.set_ylim((y_min, y_max))
    if dump:
        fig1.savefig(title + ".png")

def integrate(lst):
    total = 0
    for v in lst:
        total += v

    return total


def main(fname1, fname2, display_integration=False, fname3 = None):
    print(fname1, fname2, fname3)
    f1 = open(fname1, 'r')
    f2 = open(fname2, 'r')
    if fname3 is not None:
        f3 = open(fname3, 'r')
    idx1, time1, x1, y1, z1, roll1, pitch1, yaw1 = get_data(f1)
    idx2, time2, x2, y2, z2, roll2, pitch2, yaw2 = get_data(f2)
    if fname3 is not None:
        idx3, time3, x3, y3, z3, roll3, pitch3, yaw3 = get_data(f3)
    error = get_eucld_error(x1, x2, y1, y2, z1, z2)

    idx_x = copy.deepcopy(idx1)
    idx_y = copy.deepcopy(idx1)
    idx_z = copy.deepcopy(idx1)
    idx_error = copy.deepcopy(idx1)

    # i = 0
    # while i < len(x1):
    #     x1, x2, idx_x, i = check_val(x1, x2, idx_x, i, 100.0, -100.0)
    # i = 0
    # while i < len(y1):
    #     y1, y2, idx_y, i = check_val(y1, y2, idx_y, i, 100.0, -100.0)
    # i = 0
    # while i < len(z1):
    #     z1, z2, idx_z, i = check_val(z1, z2, idx_z, i, 100.0, -100.0)
    # i = 0
    # while i < len(error):
    #     error, idx_error, i = check_error_val(error,
    #                                           idx_error, i, 100.0, -100.0)

    if display_integration:
        print('X')
        print(integrate(x1), integrate(x2))
        print('Y')
        print(integrate(y1), integrate(y2))
        print('Z')
        print(integrate(z1), integrate(z2))
        print('Error')
        print(integrate(error)/len(error))


    plot(idx_x, x1, x2, v3=x3, title="x",
         label1=fname1.split('/')[-1].replace('.txt', ''),
         label2=fname2.split('/')[-1].replace('.txt', ''),
         label3=fname3.split('/')[-1].replace('.txt', ''),
         dump=True)
    plot(idx_y, y1, y2, v3=y3, title="y",
         label1=fname1.split('/')[-1].replace('.txt', ''),
         label2=fname2.split('/')[-1].replace('.txt', ''),
         label3=fname3.split('/')[-1].replace('.txt', ''),
         dump=True)
    plot(idx_z, z1, z2, v3=z3, title="z",
         label1=fname1.split('/')[-1].replace('.txt', ''),
         label2=fname2.split('/')[-1].replace('.txt', ''),
         label3=fname3.split('/')[-1].replace('.txt', ''),
         dump=True)
    plot_error(idx_error, error)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Display twist')
    parser.add_argument('file1')
    parser.add_argument('file2')
    parser.add_argument('--file3')
    parser.add_argument('--show_final_pose', type=bool, default=False)

    args = parser.parse_args()

    main(args.file1, args.file2, args.show_final_pose, fname3=args.file3)
