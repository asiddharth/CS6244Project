import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation
import pickle

num_cars = 5
blah = pickle.load(open("sequence_noise_0_vel_half_env55000.pkl", "rb"))

fig = plt.figure()


ax = fig.add_subplot(111, aspect = 'equal', adjustable = 'box')
ax.set_xlim(0, 200)
ax.set_ylim(10, 30)


def init():
    return []

def animate(i):
    patches = []
    for j in range(num_cars) :
        ts = ax.transData
        coords = ts.transform([blah[i][0][j][0] - 0.405,blah[i][0][j][1] - 0.6 ])
        t = 0
        if j == 0 :
            tr = mpl.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], blah[i][2])
            t = ts+tr
        else :
            t = ts
        if j == 0 :
            patch = mpatches.Rectangle((blah[i][0][j][0] - 0.405, blah[i][0][j][1] - 0.6), 2.385, 1.2, fc='r',transform=t)
        else :
            patch = mpatches.Rectangle((blah[i][0][j][0] - 0.405, blah[i][0][j][1] - 0.6), 2.385, 1.2, fc='g', transform = t)
        patches.append(ax.add_patch(patch))
    print patches
    return patches

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(blah),
                               interval=250,
                               repeat = False,
                               blit=True)
plt.ylim((10,30))
plt.xlim((0,200))
plt.show()