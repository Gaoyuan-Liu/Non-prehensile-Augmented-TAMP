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
compare_data = pd.read_csv('compare_data_all.csv')

# -------------
# Extract data
# -------------
total_episode = np.array(compare_data['total_episodes'])
without_pusher = np.array(compare_data['solve_time_without_pusher'])
with_pusher = np.array(compare_data['solve_time_with_pusher'])



# x_labels = ['without pusher', 
#             'with pusher']
# x_pos = np.arange(len(x_labels))

# # without_pusher = [without_pusher]
# # with_pusher = [with_pusher]

# results = [without_pusher/total_episode, with_pusher/total_episode]

# -----------------------
# Build the attempt plot
# -----------------------
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot([4, 5, 6, 7, 8], without_pusher, '-rD', markersize=10, linewidth=4)
ax.plot([4, 5, 6, 7, 8], with_pusher, '-gD', markersize=10, linewidth=4)
ax.set_ylabel('Success ratio (%)')
ax.set_xlabel('Number of objects')
ax.legend(['PDDLStream','PDDLStream+Pusher'])

# ---------
# Annotate
# ---------
for xi, yi in np.nditer([[4, 5, 6, 7, 8], without_pusher]):
    etiqueta = "{:.1f}".format(yi)
    ax.annotate(etiqueta, (xi,yi), xytext=(0,-30), xycoords='data', textcoords='offset points', ha="center")

for xi, yi in np.nditer([[4, 5, 6, 7, 8], with_pusher]):
    etiqueta = "{:.1f}".format(yi)
    ax.annotate(etiqueta, (xi,yi), textcoords="axes fraction",xytext=(0,10),ha="center")

# --------------
# Save the plot
# --------------

plt.savefig("compare.pdf", bbox_inches="tight")
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



