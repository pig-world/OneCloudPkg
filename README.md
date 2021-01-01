#这是什么
这是一个关于Amlogic S805的UEFI适配，很多代码都移植自S805的Uboot。

#如何编译
参考：https://www.pigworld.pw/?cat=2
- 其中ACPI和SMBIOS部分未完成，如需编译，请删除dsc文件内的引用。
- 目前还有显示部分和SD/EMMC部分的代码没有进行整理。
- 由于UEFI不支持除定时器中断外的其他中断，需要用到其他中断均需要通过定时器中断来模拟，S805的定时器中断触发在ArmGic的默认实现中有问题（可以触发但是读取不到中断源）考虑到只使用Timer中断的情况下，可采取简单粗暴的方法将触发的中断一律认作为定时器中断

#已知问题
1. HDMI颜色显示异常(已尝试多种颜色方案，均异常)
2. EMMC/SD不工作，目前写了两套实现，其一直接照搬Uboot源码（Flash文件夹），其二是根据UEFI默认的（EmbeddedPkg\Drivers\DwEmmcDxe和MdeModulePkg\Bus\Sd）进行适配，测试代码运行顺序均与Uboot一致，但是实际无法驱动，理论上这个驱动应该能同时驱动SD和EMMC，怀疑是时序问题。
3. USB偶尔不工作(原版uboot也有此问题)，如果启动之前插入usb设备则可以100%工作。

#目前进度
由于缺少相关芯片资料，特别是ACPI的实现无从下手，项目无法继续推进。如果能找到一些详细的资料，或得到某位大神的帮助，该项目是可以重启的。

#致谢
本项目参考了以下开源项目的实现，在这里在此感谢他们的无私奉献。
https://github.com/tianocore/edk2
https://github.com/WOA-Project/Lumia950XLPkg
https://github.com/150balbes/Amlogic_s8xx-kernel
https://github.com/not-aml/openlinux-amlogic-uboot