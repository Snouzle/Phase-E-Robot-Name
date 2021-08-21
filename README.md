# Phase-E-Robot-Name
MTRN4110 Phase D

Follow the instructions below to complete the execution Phase A, B and C combined:
- Open terminal and locate Phase-E-Robot-Name -> Phase_D -> programs 
- Run jupyter notebook 
- Access programs, then open z5213702_MTRN4110_PhaseC.ipynb
- Locate section 3.7, at the bottom, replace the line subprocess.Popen("C:\\.. C:\\..") with subprocess.Popen("C::\\LOCATION\\webostw.exe' C::\\ LOCATION OF FOLDER \\Phase-E-Robot-Name\\Phase_D\\worlds\\z5213702_MTRN4110_PhaseC.wbt). Where location is where the specific file located at the end of the C:\\ is located.
- Also replace the line os.startfile("C:\\Users\\nickb\\Phase-E-Robot-Name\\Phase_D\\worlds\\z5213702_MTRN4110_PhaseC.wbt") with os.startfile("C:\\ LOCATION OF FOLDER \\Phase-E-Robot-Name\\Phase_D\\worlds\\z5213702_MTRN4110_PhaseC.wbt"), where LOCATION OF FOLDER is the location where Phase-E-Robot-Name is located

Webots should now be opened with PhaseD set as the controller. To run the program, follow the instructions printed to the console.

Keyboard input is controlled by:
- 'W' for moving forward
- 'A' for turning left
- 'D' for turning right

