Fourbar_GUI
===========
Here are Matlab codes I wrote for my team project I participated at Mechanism Design class at National Taiwan University. The goal of this project is optimize the design of elliptical trainer, which is one of fitness equipment we can can see in gym.

* FourAnalysis.m : for four-bar type elliptical trainer. [Four Bar](https://youtu.be/97GhadOHFXM)
* EightbarAnalysis.m : for eight-bar type elliptical trainer. [Eight Bar](https://youtu.be/6qxCdixfiII)
* FourBarGUI.fig&m : Using Matlab GUIDE to make a GUI of four-bar linkage display. [GUI](https://youtu.be/cMqUwASI1J0)

More detail about [the project](http://chienpinchen.blogspot.tw/2011/08/team-project-optimal-design-of.html)




Requirements
------------
* MATLAB R2020a or newer is recommended. The source files are UTF-8 encoded (the Chinese comments and prompts were originally Big5), and MATLAB reads UTF-8 source files by default starting in R2020a.
* The code was updated for modern MATLAB (R2014b+ graphics): the removed `EraseMode` plot property is no longer used, and `break` outside a loop was replaced by `error`/`return`.
* `fourbarGUI` was built with GUIDE. Newer MATLAB releases no longer include the GUIDE editor, but the existing `.fig` + `.m` pair still runs: add the folder to the path and call `fourbarGUI`.
