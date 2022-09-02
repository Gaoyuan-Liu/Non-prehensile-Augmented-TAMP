import pandas as pd
import os
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import numpy as np

# Locate local working directory on where this file is
outdir = os.path.dirname(os.path.abspath(__file__))
os.chdir(outdir)

# Style
fontsize = 18
plt.rc('font', size=fontsize)  

# ------------
# Import data
# ------------
compare_data = pd.read_csv('compare_data.csv')

# -------------
# Extract data
# -------------
total_episode = np.array(compare_data['total_episodes'])[0]
without_pusher = np.array(compare_data['solve_time_without_pusher'])[0]
with_pusher = np.array(compare_data['solve_time_with_pusher'])[0]



x_labels = ['without pusher', 
            'with pusher']
x_pos = np.arange(len(x_labels))

# without_pusher = [without_pusher]
# with_pusher = [with_pusher]

results = [without_pusher/total_episode, with_pusher/total_episode]

# -----------------------
# Build the attempt plot
# -----------------------
fig, ax = plt.subplots(figsize=(5, 5))
ax.bar(x_pos, results, align='center', alpha=0.9, ecolor='orange', capsize=10)
ax.set_ylabel('Success Ratio')
ax.set_xticks(x_pos)
# plt.ylim([13, 33])
# for tick in ax.get_xticklabels():
#     tick.set_rotation(-20)
ax.set_xticklabels(x_labels)
# fig.autofmt_xdate()
# plt.figure(figsize=(8, 6), dpi=80)
plt.savefig("attempt_compare_u.pdf", bbox_inches="tight")
plt.show()


# Build the replan plot
# fig, ax = plt.subplots(figsize=(8, 5))
# ax.bar(x_pos, replan_means, yerr=replan_errors, align='center', alpha=0.5, ecolor='orange', capsize=10)
# ax.set_ylabel('Replanning request times')
# ax.set_xticks(x_pos)
# ax.set_xticklabels(x_labels)
# plt.ylim([14, 70])
# #fig.autofmt_xdate()
# plt.savefig("replan_compare_u.pdf", bbox_inches="tight")
# plt.show()



