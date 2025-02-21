import os
import sys
import pandas as pd
import numpy as np

def mttc(a_subject, v_subject, a_preceding, v_preceding, d):
    da = a_subject - a_preceding
    dv = v_subject - v_preceding
    if da != 0:
        t1 = (-dv - (dv**2 + 2*da*d)**0.5)/da
        t2 = (-dv + (dv**2 + 2*da*d)**0.5)/da
        # take just real part
        if t1.imag != 0 or t2.imag != 0:
            return np.nan

        if t1 > 0 and t2 > 0:
            if t1 >= t2:
                return t2
            elif t1 < t2:
                return t1
        elif t1 > 0 and t2 <= 0:
            return t1
        elif t1 <= 0 and t2 > 0:
            return t2
    if da == 0 and dv > 0:
        return d/dv

def ttc(v_subject, v_preceding, d):
    print(f'v_subject: {v_subject}, v_preceding: {v_preceding}, d: {d}')
    dv = v_subject - v_preceding
    return max(min(10, d/dv), -10)

def process_data(df):
    print(f'Processing {df.shape[0]} rows of data')
    # Calculate the leader based on if c1 or c2 never is zero
    preceding = 'OR' if df['c1'].min() != 0 else 'M'    # leader
    subject = 'OR' if preceding == 'M' else 'M'         # follower
    print(f'Preceding vehicle is {preceding} and subject vehicle is {subject}')

    a = 'minus' if subject == 'M' else 'plus' # acceleration
    a = f'a_{a}'

    # Calculate the time to collision at each time step
    # df['ttc'] = df.apply(lambda row: mttc(-row[a], row[f'V_{subject}'], row[a], row[f'V_{preceding}'], row['dist']), axis=1)
    df['ttc'] = df.apply(lambda row: ttc(row[f'V_{subject}'], row[f'V_{preceding}'], row['dist']), axis=1)

def generate_plots(folder_name):
    print('Generating plots')
    # Get the file path for the CSV file in the folder
    # open first .csv extension file
    file_path = [os.path.join(folder_name, f) for f in os.listdir(folder_name) if f.endswith('.csv')][0]

    # Load the CSV file into a pandas dataframe
    df = pd.read_csv(file_path)

    # Process the data in the dataframe
    process_data(df)

    # Generate plots using pandas and matplotlib
    plt = df.plot(x='time', y=['ttc'], title='TTC')
    plt.get_figure().savefig(f'{folder_name}/ttc.png')
    print(f'TTC plot saved to {folder_name}/ttc.png')

if __name__ == '__main__':
    folder_name = sys.argv[1]
    # Test the function with a sample folder name
    generate_plots(folder_name)
