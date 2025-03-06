wasn't able to get nearly as good data as original test located in 

D:\all my files\documents\uvm\5_masters\hive\github_directory\HIVE\vision

and 

D:\all my files\documents\uvm\5_masters\hive\github_directory\results\distance-accuracy\aruco_approx_dist

use the original aruco_approx_dist_2.xlsx data

**********************************************************
^^ 25/3/5 discovered this is because theta pov is wrong!! 
advertised as 87 deg on spec sheet, but 
experimentally the depth fov is actually 81.15 deg

-> after measuring the correct fov fo 81.15 and correcting 
this factor in the software, got great results
**********************************************************