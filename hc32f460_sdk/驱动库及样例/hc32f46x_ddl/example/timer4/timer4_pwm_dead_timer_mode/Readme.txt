﻿================================================================================
                                样例使用说明
================================================================================
版本历史
日期        版本    负责人         IAR     MDK   描述
2018-10-30 1.0     Hongjh         7.70    5.16  first version
================================================================================
功能描述
================================================================================
本样例主要展示如何使用Timer4PWM死区定时器模式。

说明：
样例配置比较输出模块OCO奇偶通道，并通过PWM死区定时器模式，输出其波形。

================================================================================
测试环境
================================================================================
测试用板:
---------------------
EV-HC32F460-LQFP100-050-V1.1

辅助工具:
---------------------
示波器Tektronix MDO3054

辅助软件:
---------------------
无

================================================================================
使用步骤
================================================================================
1）测试板J6-39(PWM_L)、J6-40(PWM_H)引脚与示波器相连；
2）编译、下载并运行；
3）观察示波器，J6-39、J6-40通道波形周期相同，且周期时间为CNT周期2倍。PWM_L占空比
   为1/4，PWM_H占空比为1/2
               _     _     _     _     _
   J6-40    __| |___| |___| |___| |___| |

            _    __    __    __    __
   J6-39     |__|  |__|  |__|  |__|  |__|

================================================================================
注意
================================================================================
若更换Timer4单元或Timer4 PWM通道，请连接对应的PWM引脚。

================================================================================
