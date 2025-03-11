# pmsm_sensorless_1shunt_foc

**【功能描述】**
+ 基于ECMCU105H/ECBMCU201MPC单板的单电机单电阻采样Foc应用

**【环境要求】**
+ 所有单板电源改制为演示用的12V低压，电机选用Gimbal GBM2804H-100T

**【IDE配置方法】**
+ chipConfig中的Sample栏目里面选中pmsm sensorless 1shunt foc示例，然后点击生成代码即可

**【注意事项】**
+ 此示例init文件夹下的工程初始化配置是基于3065HRPIRZ芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考SDK sample下的init文件内容；
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。
