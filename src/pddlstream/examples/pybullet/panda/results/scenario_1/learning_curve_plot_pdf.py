import pandas as pd
import os
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import numpy as np

# Locate local working directory on where this file is
outdir = os.path.dirname(os.path.abspath(__file__))
os.chdir(outdir)

# ----------------
# Import raw data
# ----------------
run = pd.read_csv('plot_data.csv')

# --------------
# Define colors
# --------------
RebeccaPurple = (102/255, 51/255, 153/255)
OrangeRed = (255/255, 69/255, 0)
YellowGreen = (154/255, 205/255, 50/255)

# -------------------
# Global definiation
# -------------------
color_1 = 'blue'
color_2 = 'green'
linewidth_1 = 2
linewidth_2 = 2
fontsize = 32
titlename = "Bin Scenario"
window = 100
# figure(figsize=(8, 6), dpi=80)



# ---------------
# Episode Length
# ---------------
el_average = pd.DataFrame({'el_ave':[]})

# Window filter data
for i in range(len(run['el'])):
    if i == 0:
        df = pd.DataFrame({'el_ave':[run['el'][i]]})
        el_average = el_average.append(df, ignore_index=True)
    else:
        windowsum = 0
        windowlist = []
        for j in range(window):
            index = i - (window%2) + j
            if index >= 0 and index < len(run['el']):
                windowlist.append(run['el'][index])
        if len(windowlist) == window:
            windowsum = sum(windowlist)
            ave = windowsum / len(windowlist)
            df = pd.DataFrame({'el_ave':[ave]})
            el_average = el_average.append(df, ignore_index=True)


el_average = el_average.iloc[1: len(run), : ].reset_index()
ax = el_average.reset_index().plot(x='index', y='el_ave', kind='line',
                        linewidth=linewidth_1,
                        fontsize=fontsize,
                        # xticks=np.arange(0, (len(run)//100)*100 + 400, step=400),
                        color=color_1)


plt.xlabel('episode', fontsize=fontsize)
plt.ylabel('Average episode length', fontsize=fontsize)
# plt.legend(['Without safety layer', 'With safety layer'], loc="upper left", bbox_to_anchor=(1.04, 1.0), fontsize=fontsize, borderaxespad=0) # loc=1,borderaxespad=0
ax.get_legend().remove()
# plt.legend(['Without safety layer', 'With safety layer'], fontsize=fontsize)
# plt.legend(['Without safety layer', 'With safety layer'], bbox_to_anchor=(1, 1.0),fontsize=fontsize, loc='upper left', borderaxespad=0)
# plt.suptitle(titlename, fontsize=fontsize)
plt.savefig("el_s1.pdf", bbox_inches="tight")
plt.show()


# ---------------
# Episode Return
# ---------------
er_average = pd.DataFrame({'er_ave':[]})


for i in range(len(run['er'])):
    if i == 0:
        df = pd.DataFrame({'er_ave':[run['er'][i]]})
        er_average = er_average.append(df, ignore_index=True)
    else:
        windowsum = 0
        windowlist = []
        for j in range(window):
            index = i - (window%2) + j
            if index >= 0 and index < len(run['er']):
                windowlist.append(run['er'][index])
        if len(windowlist) == window:
            windowsum = sum(windowlist)
            ave = windowsum / len(windowlist)
            df = pd.DataFrame({'er_ave':[ave]})
            er_average = er_average.append(df, ignore_index=True)


er_average = er_average.iloc[1: len(run), : ].reset_index()
ax = er_average.reset_index().plot(x='index', y='er_ave', kind='line',
                        linewidth=linewidth_1,
                        fontsize=fontsize,
                        xticks=np.arange(0, (len(run)//100)*100 + 100, step=1000),
                        color=color_1)

plt.xlabel('Episode', fontsize=fontsize)
plt.ylabel('Average return', fontsize=fontsize)
ax.get_legend().remove()
# plt.legend(['Without safety layer', 'With safety layer'], bbox_to_anchor=(1, 1.0),fontsize=fontsize, borderaxespad=0)

# plt.suptitle(titlename, fontsize=fontsize)
plt.savefig("er_s1.pdf", bbox_inches="tight")
plt.show()

# ----------------
# Invalid Actions
# ----------------
ia_average = pd.DataFrame({'ia_ave':[]})


for i in range(len(run['ia'])):
    if i == 0:
        df = pd.DataFrame({'ia_ave':[run['ia'][i]]})
        ia_average = ia_average.append(df, ignore_index=True)
    else:
        windowsum = 0
        windowlist = []
        for j in range(window):
            index = i - (window%2) + j
            if index >= 0 and index < len(run['ia']):
                windowlist.append(run['ia'][index])
        if len(windowlist) == window:
            windowsum = sum(windowlist)
            ave = windowsum / len(windowlist)
            df = pd.DataFrame({'ia_ave':[ave]})
            ia_average = ia_average.append(df, ignore_index=True)


ia_average = ia_average.iloc[1: len(run), : ].reset_index()
ax = ia_average.reset_index().plot(x='index', y='ia_ave', kind='line',
                        linewidth=linewidth_1,
                        fontsize=fontsize,
                        xticks=np.arange(0, (len(run)//100)*100 + 100, step=1000),
                        color=color_1)

plt.xlabel('Episode', fontsize=fontsize)
plt.ylabel('Invalide action', fontsize=fontsize)
ax.get_legend().remove()
# plt.legend(['Without safety layer', 'With safety layer'], bbox_to_anchor=(1, 1.0),fontsize=fontsize, borderaxespad=0)

# plt.suptitle(titlename, fontsize=fontsize)
plt.savefig("ia_s1.pdf", bbox_inches="tight")
plt.show()

# ---------------
# Sucess ratio
# ---------------
window = 50
sr_average = pd.DataFrame({'sr_ave':[]})


for i in range(len(run['sr'])):
    if i == 0:
        df = pd.DataFrame({'sr_ave':[run['sr'][i]]})
        sr_average = sr_average.append(df, ignore_index=True)
    else:
        windowsum = 0
        windowlist = []
        for j in range(window):
            index = i - (window%2) + j
            if index >= 0 and index < len(run['sr']):
                windowlist.append(run['sr'][index])
        if len(windowlist) == window:
            windowsum = sum(windowlist)
            ave = windowsum / len(windowlist)
            df = pd.DataFrame({'sr_ave':[ave]})
            sr_average = sr_average.append(df, ignore_index=True)


sr_average = sr_average.iloc[1: len(run), : ].reset_index()
ax = sr_average.reset_index().plot(x='index', y='sr_ave', kind='line',
                        linewidth=linewidth_1,
                        fontsize=fontsize,
                        xticks=np.arange(0, (len(run)//100)*100 + 100, step=1000),
                        color=color_1)

plt.xlabel('Episode', fontsize=fontsize)
plt.ylabel('Average return', fontsize=fontsize)
ax.get_legend().remove()
# plt.legend(['Without safety layer', 'With safety layer'], bbox_to_anchor=(1, 1.0),fontsize=fontsize, borderaxespad=0)

# plt.suptitle(titlename, fontsize=fontsize)
plt.savefig("sr_s1.pdf", bbox_inches="tight")
plt.show()
