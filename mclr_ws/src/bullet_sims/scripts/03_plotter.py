import os
import rospy
import rospkg
import csv
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

def animate(i):
  # load data
  csv_file = os.path.join(rospkg.RosPack().get_path('bullet_sims'), 'doc/data.csv')
  data = pd.read_csv(csv_file)

  time = data['time'].values
  ref_height = data['reference_height'].values
  comp_height = data['computed_height'].values
  sim_height = data['simulator_height'].values

  # plot
  plt.cla()
  plt.plot(time, ref_height, label='reference height')
  plt.plot(time, comp_height, label='computed height')
  plt.plot(time, sim_height, label='simulator height')

  plt.legend(loc='upper left')


def main():
  ani = FuncAnimation(plt.gcf(), animate, interval=1000)
  plt.tight_layout()
  plt.show()
  None

if __name__ == '__main__':
  rospy.init_node('plotter')
  if not rospy.is_shutdown():
    main()