import os
import rospy
import rospkg
import csv
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

def main():
  ########################################################################
  # animate function
  ########################################################################
  def animate(i):
    # load data
    csv_file = os.path.join(rospkg.RosPack().get_path('bullet_sims'), 'doc/grp.csv')
    data = pd.read_csv(csv_file)

    time = data['time'].values
    
    zmp_x = data['zmp_x'].values
    zmp_y = data['zmp_y'].values
    # zmp_z = data['zmp_z'].values

    cmp_x = data['cmp_x'].values
    cmp_y = data['cmp_y'].values
    # cmp_z = data['cmp_z'].values

    cp_x = data['cp_x'].values
    cp_y = data['cp_y'].values
    # cp_z = data['cp_z'].values

    # plot
    axs[0].cla()
    axs[0].plot(time, zmp_x, label='zmp_x')
    axs[0].plot(time, zmp_y, label='zmp_y')
    # axs[0].plot(time, zmp_z, label='zmp_z')
    axs[0].legend(loc='upper left')

    axs[1].cla()
    axs[1].plot(time, cmp_x, label='cmp_x')
    axs[1].plot(time, cmp_y, label='cmp_y')
    # axs[1].plot(time, cmp_z, label='cmp_z')
    axs[1].legend(loc='upper left')

    axs[2].cla()
    axs[2].plot(time, cp_x, label='cp_x')
    axs[2].plot(time, cp_y, label='cp_y')
    # axs[2].plot(time, cp_z, label='cp_z')
    axs[2].legend(loc='upper left')
  ########################################################################

  fig, axs = plt.subplots(3, 1)
  if not rospy.is_shutdown():
    ani = FuncAnimation(fig, animate, interval=1000)
    plt.tight_layout()
    plt.show()
    None

if __name__ == '__main__':
  rospy.init_node('plotter')
  main()