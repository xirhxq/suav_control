import pandas as pd
import matplotlib.pyplot as plt
from glob import glob

csvList = glob('2023-09-04-*_search.csv')

for csv in csvList:
    try: 
        df = pd.read_csv(csv)
    except:
        continue
    plt.figure()
    plt.plot(df['taskTime'], df['pos.z'], '*', label='Height')
    plt.savefig(csv[:-4] + '_height.png')
