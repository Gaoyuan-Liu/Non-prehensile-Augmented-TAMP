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
n_file = 2

plot_date_0 = pd.read_csv('compare_data_44.csv')
plot_date_1 = pd.read_csv('compare_data_55.csv')
plot_date_2 = pd.read_csv('compare_data_66.csv')
plot_date_3 = pd.read_csv('compare_data_77.csv')
plot_date_4 = pd.read_csv('compare_data_88.csv')



compare_data_all = plot_date_0.append(plot_date_1, ignore_index=True)
compare_data_all = compare_data_all.append(plot_date_2, ignore_index=True)
compare_data_all = compare_data_all.append(plot_date_3, ignore_index=True)
compare_data_all = compare_data_all.append(plot_date_4, ignore_index=True)

compare_data_all.to_csv('compare_data_all.csv')