# task
added task1 and task2

Day Wise work distribution:

Wednesday:
- Got to know about various drone firmwares such as PX4 and Ardupilot, selected Ardupilot.
- Researched on supporting libraries such as dronekit and mavros, used both of them.
- Installed the ardupilot, dronekit, mavros, mavlink and SITL
- Found out that Gazebo was somehow removed from my system
- Couldn't install gazebo11 due to an error in the gazebo.deb file

Thursday:
- Trying to sort out the error of Gazebo installation
- Removal of Gazebo also stopped midway, due to error in the gazebo.deb file
- Removal of ROS ecosystem also stopped midway, due to error in the gazebo.deb file

Friday:
- Found out that the entire gazebo packages were broken, and couldn't install any other packages which were not even Gazebo related.
- Removed the entire Ubuntu dual boot system
- Dual booted a fresh Ubuntu 20.04 iso file.

Saturday:
- Installed ROS Noetic, and all the drone related softwares again.
- Installed Gazebo Garden which is operated using gz_sim, and is different from the classic Gazebo. Apparently Gazebo9 and Gazebo11 didn't work with Ardupilot.
- Read the code documentation for dronekit library, and wrote the code for takeoff and land.
- Completed the task1.

Sunday and Monday:
- Worked on task2.
- Worked on helical trajectory code in mavros python.
- Same code worked in mavros but not in dronekit.
- Completed the task2

Tuesday:
- Couldn't work on the task3, due to some unavoidable situations.

Wednesday:
- Started task3, trying to bring in camera plugin in gz sim since the drone isn't equipped with camera.
- Installed Gazebo11 since the classic Gazebo plugins don't work in gz_sim.
- Installed fpv camera plugin and aruco marker in the world file.

Thursday and Friday:
- Implemented the code, took help from another github repo for helper functions.
- Currently facing error, not able to connect vehicle to ardupilot gazebo through code.
- Main detection and landing code seems correct.

Monday - Wednesday
- Solved the above error by increasing the timeout parameter.
- Reframed the code, and removed the error causing lines.
- Code works fine, not suitable for fast movement of aruco marker.

Task1 youtube video: https://youtu.be/zQm7lfQaEe0?si=s73Zmwa72Us4VghH

Task2 youtube video: https://youtu.be/Mi4Nh4uRUyM?si=liefssOqN2AcrbGO

Task3(working) youtube video: https://youtu.be/6vbPx5Ymrvs?si=ChdRBVP80BvzM6h-
