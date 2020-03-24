import matplotlib.pyplot as plt
import copy

import argparse


def get_data(f, ignore_count = []):
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
        #if float(line.split(',')[1]) < 0.25 and float(line.split(',')[1]) > -.25:
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
        if l1[ind] > 0.75:
            print(l1[ind], ind)

        l1.pop(ind)
        l2.pop(ind)

        idx.pop(ind)
    elif l1[ind] < min_val or l2[ind] < min_val:
        l1.pop(ind)
        l2.pop(ind)
        idx.pop(ind)
    else:
        ind+=1

    return l1, l2, idx, ind

def plot(idx_list, v1, v2, title = " ", label1 = "1", label2 = "2", dump=False):
    fig1, ax1 = plt.subplots()
    #print(v1)
    ax1.scatter(idx_list, v1, c='k', label = label1)
    ax1.scatter(idx_list, v2, c='b', label = label2)
    ax1.set_title(title)
    ax1.legend()
    if dump:
        fig1.savefig(title + ".png")

def main(fname1, fname2):
    f1 = open(fname1, 'r')
    f2 = open(fname2, 'r')
    idx1, time1, x1, y1, z1, roll1, pitch1, yaw1 =  get_data(f1)
    idx2, time2, x2, y2, z2, roll2, pitch2, yaw2 =  get_data(f2)
    idx_x = copy.deepcopy(idx1)
    idx_y = copy.deepcopy(idx1)
    idx_z = copy.deepcopy(idx1)
    i = 0
    while i < len(x1):
        x1, x2, idx_x, i = check_val(x1, x2, idx_x, i, 0.25, -0.25)
    i = 0
    while i < len(y1):
        y1, y2, idx_y, i = check_val(y1, y2, idx_y, i, 0.25, -0.25)
    i = 0
    while i < len(z1):
        z1, z2, idx_z, i = check_val(z1, z2, idx_z, i, 0.25, -0.25)


    #print(i, len(x1))
    plot(idx_x, x1, x2, title="x",
                label1 = fname1.split('/')[-1].replace('.txt', ''),
                label2 = fname2.split('/')[-1].replace('.txt', ''), dump=True)
    plot(idx_y, y1, y2, title="y",
                label1 = fname1.split('/')[-1].replace('.txt', ''),
                label2 = fname2.split('/')[-1].replace('.txt', ''), dump=True)
    plot(idx_z, z1, z2, title = "z",
                label1 = fname1.split('/')[-1].replace('.txt', ''),
                label2 = fname2.split('/')[-1].replace('.txt', ''), dump=True)
    # fig2, ax2 = plt.subplots()
    # ax2.scatter(idx_y, y1, c='k')
    # ax2.scatter(idx_y, y2, c='b')
    # ax2.set_title("y")
    #
    # fig3, ax3 = plt.subplots()
    # ax3.scatter(idx_z, z1, c='k')
    # ax3.scatter(idx_z, z2, c='b')
    # ax3.set_title("z")
    plt.show()


parser = argparse.ArgumentParser('Display twist')
parser.add_argument('file1')
parser.add_argument('file2')

args = parser.parse_args()

main(args.file1, args.file2)
