Matlab device class for controlling a Keithley 2450 sourcemeter over TSP
This project contains a device class for controlling a Keithley 2450 (TSP) sourcemeter in Matlab (Thanks to Tektronix for CHANGING MOST BASIC FUNCTIONS IN SCPI with no backward compability). The implemented methods include standard current-voltage measurements, time resolved current-voltage sweep measurements, time resolved current-density point measurements, cyclic current-voltage measurements (MPP->JSC->VOC->JSC), and steady state tracking measurements of maximum power point, VOC, and JSC.

The main function is classKeithley2450.m, which will create a device object for the Keithley 2450 (TSP). There are two optional input parameters (string connectionType, string/int port), which can be also set after creation. 

All methods are implemented into classKeithley2400_testscript.m (see my other repository) and can be tested seperately. For running the program, a GPIB controller is required and the Keithley has to be set to TSP. A serial connection does not work and all regarding settings will be removed in the future.

This is an initial release. Not all funcitons are already operating and will be added with time.

Tested: Matlab 2018a, Win10, NI GPIB-USB-HS+ Controller, Keithley 2450

Due to a significant change in graphics handling with Matlab 2014b this program is NOT COMPATIBLE TO MATLAB 2014a AND BELOW.

Author: Eugen Zimmermann, Konstanz, 2016 eugen.zimmermann [at] uni-konstanz [dot] de

Based on publication:
Characterization of perovskite solar cells: Towards a reliable measurement protocol
Eugen Zimmermann, Ka Kan Wong, Michael Müller, Hao Hu, Philipp Ehrenreich, Markus Kohlstädt, Uli Würfel, Simone Mastroianni, Gayathri Mathiazhagan, Andreas Hinsch, Tanaji P. Gujar, Mukundan Thelakkat, Thomas Pfadler, and Lukas Schmidt-Mende
http://dx.doi.org/10.1063/1.4960759

Last Modified on 2018-08-20

ToDo:
- remove everything related to serial connection
- finish and test functions

Version 1.0
- initial release