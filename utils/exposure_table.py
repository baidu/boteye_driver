#!/usr/bin/env python
# This script calculates the exposure / gain table with two different approaches:
# 1. Constant brightness delta
# 2. Constant brightness ratio
#
# The max exposure is set to 1024 (2^10)

import pandas as pd
import numpy as np
import itertools
import matplotlib.pyplot as plt
import os

viz=True

# 1. Constant delta step for brightness
#    The analog gain step of V024 is 1/16
gain_list = [1 + i/16.0 for i in range(16)]
exp_list = [pow(2,i) for i in range(11)]
gain_exp_list = list(itertools.product(exp_list, gain_list))

df = pd.DataFrame(index = np.arange(len(gain_exp_list)))
df['gain'] = [item[1] for item in gain_exp_list]
df['exp'] = [item[0] for item in gain_exp_list]
df['gain_x_exp'] = df['gain'] * df['exp']
df['gain_x_exp_1'] = df['gain_x_exp'].shift()
df['inc_ratio'] = df['gain_x_exp'] / df['gain_x_exp_1']
df['inc_percentage'] = (df['inc_ratio'] - 1) * 100
df.to_csv('/tmp/constant_delta.csv')

# 2. Constant ratio step for brightness
#    gain_steps = [1, r, r^2, r^3, ... , r^n], where r^n < 2
step_ratio = 1.03
gain_steps = int(np.log(2) / np.log(step_ratio)) + 1
df_inverse = pd.DataFrame(index = np.arange(gain_steps * 11))

exp_list_multiplier = [pow(1.03, i) for i in range(gain_steps)]
df_inverse['gain_x_exp'] = [item[0] * item[1] for item in itertools.product(exp_list, exp_list_multiplier)]
df_inverse['exp'] = np.repeat(np.power(2, np.arange(11)), gain_steps)
df_inverse['gain'] = df_inverse['gain_x_exp'] / df_inverse['exp']
df_inverse.to_csv('/tmp/constant_ratio.csv')

# 2.1. We need to quantize the gain value to a step of 1/16
df_quantized = df_inverse.copy();
df_quantized['gain'] = (df_quantized['gain'] * 16 + 0.5).astype('int') / 16.0
df_quantized['gain_x_exp'] = df_quantized['gain'] * df_quantized['exp']
df_quantized['gain_x_exp_1'] = df_quantized['gain_x_exp'].shift()
df_quantized['inc_ratio'] = df_quantized['gain_x_exp'] / df_quantized['gain_x_exp_1']
df_quantized['inc_percentage'] = (df_quantized['inc_ratio'] - 1) * 100
if viz:
    plt.figure()
    plt.suptitle('fixed inc ratio gain/exp table')
    plt.subplot(3, 1, 1)
    plt.plot(np.arange(len(df_quantized)), df_quantized['gain'], 'b')
    plt.plot(np.arange(len(df_inverse)), df_inverse['gain'], 'r')

    plt.subplot(3, 1, 2)
    plt.plot(np.arange(len(df_quantized)), df_quantized['inc_percentage'], 'b')

    plt.subplot(3, 1, 3)
    plt.plot(np.arange(len(df_quantized)), df_quantized['gain_x_exp'], 'b')
    plt.plot(np.arange(len(df_inverse)), df_inverse['gain_x_exp'], 'r')

# 2.2. Remove the duplicated gain_x_exp rows after quantization
df_quantized_new = df_quantized.groupby(['gain_x_exp']).first().reset_index()
df_quantized_new.to_csv('/tmp/constant_quantized_ratio.csv')
if viz:
    plt.figure()
    plt.suptitle('final gain/exp table')
    plt.subplot(3, 1, 1)
    plt.plot(np.arange(len(df_quantized_new)), df_quantized_new['inc_percentage'])
    plt.ylabel('inc_percentage(%)')
    ax1 = plt.subplot(3, 1, 2)
    ax1.plot(np.arange(len(df_quantized_new)), df_quantized_new['gain'], 'b')
    ax1.set_ylabel('gain', color='b')
    ax1.tick_params('y', colors='b')
    ax2 = ax1.twinx()
    ax2.plot(np.arange(len(df_quantized_new)), df_quantized_new['exp'], 'r')
    ax2.set_ylabel('exp', color='r')
    ax2.tick_params('y', colors='r')

    plt.subplot(3, 1, 3)
    plt.plot(np.arange(len(df_quantized_new)), df_quantized_new['gain_x_exp'])
    plt.ylabel('gain_x_exp')

# 3. Convert the gain/exp values to true register values and
#    add extra steps to max out gain after exposure is maxed out
df_aec_table = pd.DataFrame()
df_aec_table['gain_reg'] = (df_quantized_new['gain'] * 16).astype('int')
df_aec_table['exp'] = df_quantized_new['exp'].astype('int')

extra_aec_rows = []
max_exp = df_aec_table['exp'].iloc[-1]
gain = df_quantized_new['gain'].iloc[-1]
gain_reg_val = df_aec_table['gain_reg'].iloc[-1]
new_gain_reg_val = gain_reg_val
while True:
    new_gain_reg_val += 1
    new_gain = new_gain_reg_val / 16.0

    # Max analog gain is 4 (reg_val=64) for V024
    if (new_gain_reg_val == 64):
        extra_aec_rows.append([new_gain_reg_val, max_exp])
        break
    if new_gain / gain >= step_ratio:
        # Create a new row for df_aec_table
        extra_aec_rows.append([new_gain_reg_val, max_exp])
        gain_reg_val = new_gain_reg_val
        gain = new_gain

df_extra_aec_table = pd.DataFrame(extra_aec_rows, columns=['gain_reg', 'exp'])
df_aec_table = df_aec_table.append(df_extra_aec_table, ignore_index=True)
df_aec_table.to_csv('/tmp/aec_table.csv')

# 4. Generate a *NEW* gain exposure table that both gain and exposure increase monotonically.
# The problem with the table in Step 3 is that we may adjust them in different directions.
# The gain reg change takes effect at frame n+1, while the exp reg change takes effect at frame n+2
# This will cause a one-frame blink.  For example:
#       frame n  -->  frame n+1  -->  frame n+2
# gain     32            17              17
#  exp     64            64             128
#
# With the monotonic increase constraint, it's impossible to maintain the fixed ratio property
# for the first few rows.  We have to sacrify the large ratio step when exp_reg_val is close to 0
# Hardcoded first couple rows that doesn't follow the fixed ratio rule
mono_aec_rows = []
gain_reg_val = 16;  # for V024
exp_reg_val = 1
exp_seg_pnts =[1, 16, 32, 64, 64+32, 128, 128+64, 256, 256 + 128, 512, 1024]

for i in range(len(exp_seg_pnts)-1):
    for val in range(exp_seg_pnts[i], exp_seg_pnts[i+1]):
        if (exp_reg_val == 1 or
            float(val) / exp_reg_val > 1.03):
            exp_reg_val = val
            mono_aec_rows.append([gain_reg_val, exp_reg_val])
    gain_reg_val += 1
    mono_aec_rows.append([gain_reg_val, exp_reg_val])

new_gain_reg_val = gain_reg_val
while True:
    new_gain_reg_val += 1

    # Max analog gain is 4 (reg_val=64) for V024
    if new_gain_reg_val == 64:
        mono_aec_rows.append([new_gain_reg_val, exp_reg_val])
        break
    if float(new_gain_reg_val) / gain_reg_val >= step_ratio:
        mono_aec_rows.append([new_gain_reg_val, exp_reg_val])
        gain_reg_val = new_gain_reg_val

df_mono_aec_table = pd.DataFrame(mono_aec_rows, columns=['gain_reg', 'exp'])
df_mono_aec_table['gain_x_exp'] = df_mono_aec_table['gain_reg'] / 16.0 * df_mono_aec_table['exp']
df_mono_aec_table['gain_x_exp_1'] = df_mono_aec_table['gain_x_exp'].shift()
df_mono_aec_table['inc_ratio'] = df_mono_aec_table['gain_x_exp'] / df_mono_aec_table['gain_x_exp_1']
df_mono_aec_table['inc_percentage'] = (df_mono_aec_table['inc_ratio'] - 1) * 100
df_mono_aec_table.to_csv('/tmp/mono_aec_table.csv')
if viz:
    plt.figure()
    plt.suptitle('mono gain/exp table')
    ax0 = plt.subplot(3, 1, 1)
    ax0.plot(np.arange(len(df_mono_aec_table)), df_mono_aec_table['inc_percentage'])
    ax0.set_ylabel('inc_percentage(%)')
    ax0.set_ylim([0, 10])
    ax1 = plt.subplot(3, 1, 2)
    ax1.plot(np.arange(len(df_mono_aec_table)), df_mono_aec_table['gain_reg']/16.0, 'b')
    ax1.set_ylabel('gain', color='b')
    ax1.tick_params('y', colors='b')
    ax2 = ax1.twinx()
    ax2.plot(np.arange(len(df_mono_aec_table)), df_mono_aec_table['exp'], 'r')
    ax2.set_ylabel('exp', color='r')
    ax2.tick_params('y', colors='r')

    plt.subplot(3, 1, 3)
    plt.plot(np.arange(len(df_mono_aec_table)), df_mono_aec_table['gain_x_exp'])
    plt.ylabel('gain_x_exp')
    plt.show()

# 5. Convert the gain/exp table to c++ code
BANNER = """/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
"""

HEADER = """
/*
 * [NOTE] This file is automatically generated by exposure_table.py
 * Do NOT modify this file directly!
 */
#ifndef INCLUDE_DRIVER_XP_AEC_TABLE_H_
#define INCLUDE_DRIVER_XP_AEC_TABLE_H_

namespace XPDRIVER {
namespace XP_SENSOR {

"""

FOOTER = """
}  // namespace XP_SENSOR
}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_AEC_TABLE_H_
"""
curr_dir = os.path.dirname(os.path.abspath(__file__))
tgt_path = curr_dir + "/../include/driver/xp_aec_table.h" ##submodule driver
tgt_path = os.path.abspath(tgt_path)
df_output_table = df_mono_aec_table

N_steps = len(df_output_table)
f = open(tgt_path, 'w')
f.write(BANNER)
f.write(HEADER)
f.write("const uint32_t kAEC_steps = {};\n".format(N_steps))
f.write("// Each row in kAEC_LUT is [gain_reg_val, exp_reg_val]\n")
f.write("const int16_t kAEC_LUT[{}][2] = {{\n".format(N_steps))
for i in range(N_steps - 1):
    gain_reg_val = df_output_table["gain_reg"][i]
    exp_reg_val = df_output_table["exp"][i]
    f.write("  {{{}, {}}},\n".format(gain_reg_val, exp_reg_val))  # double to escape curly braces
f.write("  {{{}, {}}}\n".format(df_output_table["gain_reg"].iloc[-1], df_output_table["exp"].iloc[-1]))
f.write("};\n")
f.write(FOOTER)
f.close()
