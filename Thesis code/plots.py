#!/usr/bin/env python3
import re
import os
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics


def parse_file(filename):
    pattern = re.compile(
        r'\[\s*([+-]?\d*\.?\d+)\s*\]\s*'
        r'([+-]?\d*\.?\d+)\s*,\s*'
        r'([+-]?\d*\.?\d+)\s*,\s*'
        r'([+-]?\d*\.?\d+)\s*'
    )

    times, rolls, controls = [], [], []
    with open(filename, 'r') as f:
        for line in f:
            m = pattern.match(line)
            if not m:
                continue
            t_str, r_str, u_str, _ = m.groups()
            times.append(float(t_str))
            rolls.append(float(r_str))
            controls.append(float(u_str))

    rates = []
    for i in range(len(times)):
        if i == 0:
            rates.append(0.0)
        else:
            dt = times[i] - times[i-1]
            rates.append((rolls[i] - rolls[i-1]) / dt if dt > 0 else 0.0)

    return times, rolls, controls, rates

def main():
    parser = argparse.ArgumentParser(
        description="Plot roll angle, control input, and roll rate vs time from TXT logs."
    )
    parser.add_argument('files', nargs='+', help="one or more .txt files to parse")
    parser.add_argument(
        '--trim', '-t', type=float, default=0.0,
        help="seconds to trim off the start of each dataset (default: 0.0)"
    )
    args = parser.parse_args()

    fig, axes = plt.subplots(2, 1, sharex=False, figsize=(10, 7))
    ax_roll, ax_ctrl = axes #, ax_rate

    for fname in args.files:
        t, roll, ctrl, rate = parse_file(fname)

        trim = args.trim
        idx0 = next((i for i, ti in enumerate(t) if ti >= trim), len(t))
        kept = t[idx0] if idx0 < len(t) else None
        print(f"[{os.path.basename(fname)}] trim={trim}s → dropping {idx0} samples; first kept time: {kept}")

        t2 = [ti + 0.5 for ti in t[idx0:]]  # Adjust time to start at 0s
        roll2 = roll[idx0:]
        ctrl2 = ctrl[idx0:]
        rate2 = rate[idx0:]

        window_size = 20
        smoothed = pd.Series(roll2).rolling(window=window_size,center=True).mean()

        meane = statistics.mean(ctrl2)
        
        ax_roll.plot(t2, roll2)
        # ax_roll.plot(t2, smoothed, color='green', linestyle='--', label="General behavior")
        ax_ctrl.plot(t2, ctrl2)
        # ax_rate.plot(t2, rate2, label=label)

    for ax in (ax_roll, ax_ctrl): #, ax_rate):
        ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth=0.7)
        ax.grid(which='minor', linestyle=':', linewidth=0.5)


    ax_roll.set_ylabel("Roll angle [°]")
    ax_roll.set_xlabel("Time [s]")
    ax_roll.set_title("Aircraft's response")
    ax_roll.set_ylim(bottom=-2, top=12)
    ax_roll.set_xlim(left=0, right=20)

    ax_roll.set_xticks(np.arange(0, 20.1, 2))
    
    ax_roll.axhline(0, color='black', linestyle='--', linewidth=1, label="Original setpoint")
    ax_roll.axhline(10, color='red', linestyle='--', linewidth=1, label="Second setpoint")
    # ax_roll.axhline(0, color='black', linestyle='--', linewidth=1, label="ϕ = 0◦")
    # ax_roll.axhline(20, color='red', linestyle='--', linewidth=1, label = "ϕ = 20◦")
    # ax_roll.axhline(-20, color='red', linestyle='--', linewidth=1, label = "ϕ = -20◦")


    ax_roll.legend(loc='best')

    ax_ctrl.set_ylabel("Aileron deflection [°]")
    ax_ctrl.set_xlabel("Time [s]")
    ax_ctrl.set_title("Control Input")
    # ax_ctrl.axhline(meane, color='green', linestyle='--', linewidth=1, label="Mean control value")
    ax_ctrl.set_ylim(bottom=-20.6, top=20.6)
    ax_ctrl.set_yticks(np.arange(-20, 20.1, 5))
    ax_ctrl.set_xlim(left=0, right=20)
    ax_ctrl.set_xticks(np.arange(0, 20, 2))
  


#
    # ax_ctrl.legend(loc='best')

    # ax_rate.set_ylabel("Roll rate (°/s)")
    # ax_rate.set_title("Roll Rate")
    # ax_rate.set_xlabel("Time (s)")
    # ax_rate.set_ylim(bottom=-30, top=30)
    # ax_rate.legend(loc='best')


    plt.tight_layout()
    plt.show()

main()