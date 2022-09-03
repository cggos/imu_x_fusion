# RTK

---

## Overview

<p align="center">
  <img src="../img/rtk.png">
</p>

DGPS stands for Differential GPS. It is the GPS signal corrected by ground reference stations (which can estimate the ionosphere error on the GPS signal). Traditionally, DGPS accuracy can go down to sub-meter level. RTK technique allows DGPS accuracy to be at centimeter-level.

RTK stands for Real Time Kinematic. RTK algorithms make use of the carrier phase measurement to estimate accurately the distance from the receiver to the satellites. It allows an accuracy of 1-2 cm on position.

## GPS定位 vs 差分技术

GPS定位的基本原理是：至少测量出当前位置与4颗卫星的距离和对应时间，通过解方程即可求得当前位置在地球坐标系下的位置[x, y, z]。为什么是4颗呢？因为除了需要确定[x, y, z]三个位置参数，还需要一个卫星提供相对零点时间从而消除时间误差。普通GPS的定位精度大于1米，信号误差有50%的概率会达到2米以上。另外，GPS无法支持精准定高，误差可能高达十几米。

差分就是把GPS的误差想方设法分离出。在已知位置的参考点上装上移动基站，就能知道定位信号的偏差。将这个偏差发送给需要定位的移动站，移动站就可以获得更精准的位置信息。

RTK (Real Time Kinematic), 即载波相位差分技术，它能够实时地提供测站点在指定坐标系中的三维定位结果，并达到厘米级精度。在RTK作业模式下，基站采集卫星数据，并通过数据链将其观测值和站点坐标信息一起传送给移动站，而移动站通过对所采集到的卫星数据和接收到的数据链进行实时载波相位差分处理（历时不足一秒），得出厘米级的定位结果。
