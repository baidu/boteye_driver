#!/usr/bin/env python
# This script calculates the exposure / gain table with two different approaches:
#

import pandas as pd
import numpy as np
import itertools
import matplotlib.pyplot as plt
import os
import math

viz=True

# 1. AR0141 analog gain is 2^coarse_gain *(1 + fine_gain)
# coarse_gain range is 0 - 7
# fine_gain rane is 0 - 15
# But ON semiconductor recommends limiting maximum analog gain up to 12x
max_gain = 8
# ar0141 exposure assume 400 as 25Hz
max_exposure = 500
gain_list = []
gain_reg_dict = {}
coarse_gain = [0, 1, 2, 3]
fine_gain = [i / 16.0 for i in range(16)]
for i in coarse_gain:
    for val in fine_gain:
      gain = pow(2, i) * (val + 1)
      if gain < max_gain:
        gain_list.append(gain)
        gain_reg_dict[gain] = i << 4 | (int(val * 16) & 0x0F)
      else:
        break

exp_list = [1 + i for i in range(max_exposure)]

# 2. list all possile combination between exposure and gain then filter
gain_exp_list = list(itertools.product(exp_list, gain_list))
# print(gain_exp_list)
df = pd.DataFrame(index = np.arange(len(gain_exp_list)))
df['gain'] = [item[1] for item in gain_exp_list]
df['exp'] = [item[0] for item in gain_exp_list]
df['gain_x_exp'] = df['gain'] * df['exp']
df['gain_exp_RSME'] = df['gain'] * df['gain']  + df['exp'] * df['exp']
df.to_csv('/tmp/ar0141_all_gain_exp.csv')

# 3. sort and delete reduplicated
df_tmp = df.sort_values(ascending=True, by=['gain_x_exp', 'gain_exp_RSME'])
df_tmp.to_csv('/tmp/ar0141_df_tmp.csv')
df_new = df_tmp.drop_duplicates(['gain_x_exp']).reset_index()
df_new.to_csv('/tmp/ar0141_df_new.csv')

# 4. calculate target gain_x_exp and use it filter df_new
step_ratio = 1.03
gain_x_exp_steps = int(np.log(max_gain * max_exposure) / np.log(step_ratio)) + 1
print('gain_x_exp_steps value: ', gain_x_exp_steps)
gain_x_exp_target = [pow(1.03, i) for i in range(gain_x_exp_steps)]
aec_table = pd.DataFrame()
append_aec_rows = []
i = 0
for target in gain_x_exp_target:
    left = 0
    right = len(df_new['gain_x_exp'])-1
    target_index = 0
    while left <= right:
        if right - left == 1:
            if abs(df_new['gain_x_exp'][right] - target) <= abs(df_new['gain_x_exp'][left] - target):
                target_index = right
                # print('found! right:', target_index, 'value: ', df_new['gain_x_exp'][target_index])
                break
            else:
                target_index = left
                # print('found! left:', target_index, 'value: ', df_new['gain_x_exp'][target_index])
                break
        else:
            mid = (left + right) / 2
            if i == 230:
                print('left:', left, 'right:', right, 'mid:', mid)
            if target == df_new['gain_x_exp'][mid]:
                target_index = mid
                # print('found! mid:', target_index, 'value: ', df_new['gain_x_exp'][target_index])
                break
            elif target < df_new['gain_x_exp'][mid]:
                if mid - left > 1:
                    right = mid - 1
                else:
                    right = mid
            else:
                if right - mid > 1:
                    left = mid + 1
                else:
                    left = mid
    # print(target_index)
    new_gain = df_new['gain'][target_index]
    new_exp = df_new['exp'][target_index]
    # print('append gain: ', new_gain, 'exp: ', new_exp, 'g_x_p: ',new_gain * new_exp ,'targe: ', target)
    append_aec_rows.append([new_gain, new_exp, target])
# 5. Get final gain and exposure after filter duplication then convert to gain_reg
df_append_aec_table = pd.DataFrame(append_aec_rows, columns=['gain', 'exp','target'])
df_append_aec_table['gain_x_exp'] = df_append_aec_table['gain'] * df_append_aec_table['exp']
df_append_aec_table.to_csv('/tmp/df_append_aec_table.csv')
df_aec_table = df_append_aec_table.drop_duplicates(['gain_x_exp']).reset_index()
df_aec_table['gain_x_exp_1'] = df_aec_table['gain_x_exp'].shift()
df_aec_table['inc_ratio'] = df_aec_table['gain_x_exp'] / df_aec_table['gain_x_exp_1']
df_aec_table['inc_percentage'] = (df_aec_table['inc_ratio'] - 1) * 100
# convert gain to gain_reg
df_aec_table['gain_reg'] = 0
for i in range(len(df_aec_table['gain'])):
    key = df_aec_table['gain'][i]
    df_aec_table.loc[i,'gain_reg']= gain_reg_dict[key]
df_aec_table.to_csv('/tmp/df_aec_table.csv')

if viz:
    plt.figure()
    plt.suptitle('AR0141_aec_table')
    plt.subplot(3, 1, 1)
    plt.plot(np.arange(len(df_aec_table)), df_aec_table['inc_percentage'])
    plt.ylabel('inc_percentage(%)')
    ax1 = plt.subplot(3, 1, 2)
    ax1.plot(np.arange(len(df_aec_table)), df_aec_table['gain'], 'b')
    ax1.set_ylabel('gain', color='b')
    ax1.tick_params('y', colors='b')
    ax2 = ax1.twinx()
    ax2.plot(np.arange(len(df_aec_table)), df_aec_table['exp'], 'r')
    ax2.set_ylabel('exp', color='r')
    ax2.tick_params('y', colors='r')
    ay1 = plt.subplot(3, 1, 3)
    ay1.plot(np.arange(len(df_append_aec_table)), df_append_aec_table['gain_x_exp'])
    ay1.set_ylabel('gain_x_exp', color='b')
    ay1.tick_params('y', colors='b')
    ay2 = ay1.twinx()
    ay2.plot(np.arange(len(df_append_aec_table)), df_append_aec_table['target'], 'r')
    ay2.set_ylabel('gain_x_exp_target', color='r')
    ay2.tick_params('y', colors='r')
    plt.show()

# 6. Convert the gain/exp table to c++ code
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
 * [NOTE] This file is automatically generated by exposure_table_ar0141.py
 * Do NOT modify this file directly!
 */
#ifndef INCLUDE_DRIVER_AR0141_AEC_TABLE_H_
#define INCLUDE_DRIVER_AR0141_AEC_TABLE_H_

namespace XPDRIVER {
namespace XP_SENSOR {

"""

FOOTER = """
}  // namespace XP_SENSOR
}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_AR0141_AEC_TABLE_H_
"""
curr_dir = os.path.dirname(os.path.abspath(__file__))
tgt_path = curr_dir + "/../include/driver/AR0141_aec_table.h" ##submodule driver
tgt_path = os.path.abspath(tgt_path)
df_output_table = df_aec_table

N_steps = len(df_output_table)
f = open(tgt_path, 'w')
f.write(BANNER)
f.write(HEADER)
f.write("const uint32_t kAR0141_AEC_steps = {};\n".format(N_steps))
f.write("// Each row in kAR0141_AEC_LUT is [gain_reg_val, exp_reg_val]\n")
f.write("const int16_t kAR0141_AEC_LUT[{}][2] = {{\n".format(N_steps))
for i in range(N_steps - 1):
    gain_reg_val = df_output_table["gain_reg"][i]
    exp_reg_val = df_output_table["exp"][i]
    f.write("  {{{}, {}}},\n".format(gain_reg_val, exp_reg_val))  # double to escape curly braces
f.write("  {{{}, {}}}\n".format(df_output_table["gain_reg"].iloc[-1], df_output_table["exp"].iloc[-1]))
f.write("};\n")
f.write(FOOTER)
f.close()
