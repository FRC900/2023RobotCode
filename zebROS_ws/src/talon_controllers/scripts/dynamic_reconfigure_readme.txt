Previous:
Run talon servers 1 and 2 as well as the client.
Then run rqt_reconfigure rqt_reconfigure.
You should see a steering and speed topic within rqt reconfigure.
You can adjust the values within rqt reconfigure and the values will be reflected accordingly.




Edited:
Run simlaunch in rosstd.
This runs a simulation of the robot and in turn simulates all of the topics etc to be visible in rqt reconfigure
Run rosrun rqt_reconfigure rqt_reconfigure in zebros/src
This launches a menu that allows you to dynamically reconfigure topics.
Launch "run_this_client.py" via rosrun talon_controllers run_this_client.py within zebros/src
Then run 1st_talon_server.py and 2nd_talon_server.py in zebros/src.
Via, rosrun talon_controllers 1st_talon_server.py etc after rosstd.

Remember to rosstd in each terminal.
