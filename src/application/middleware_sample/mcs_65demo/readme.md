# Motorcontrolsystem

**【功能描述】**
+ 基于AD105HDMA_VA单板的电机双FOC应用sample

**【环境要求】**
+ 所用单板电源改制为演示用的24V低压，电机选用深圳杰美康机电的42JSF630AS-1000

**【IDE配置方法】**
+ chipConfig中的sample栏目里面选中Motorcontrolsystem示例，然后点击生成代码即可

**【注意事项】**
+ 供电电源24V
+ 需在IDE的“工程”->“工程配置”->"编译优化等级"中将优化等级设为O3，否则可能会由于计算资源不够导致电机控制失败。