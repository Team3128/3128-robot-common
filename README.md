# Team 3128: Aluminum Narwhals Robot Code (Java)

This codebase is the product of years of development of Team 3128's robot code. Classes that are shared by all year-specific robot code were compiled into this codebase at the end of the 2016 season.

This codebase is divided into three sections:
+ the **Hardware Library** uses both WPILib classes and homegrown classes in concert to control more complicated assemblies
+ the **Listener Manager** runs during teleop, constantly polling buttons and joysticks, to make lambda and function calls
+ the **Autonomous Library** uses WPILib commands and the hardware library to control the robot at a higher level during the autonomous period
