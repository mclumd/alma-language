# alma_ros# alma_ros

Be aware that the ALMA\_BIN var in alma_node.py has been changed to
point to where alma is on my own local comp. Please remember to change
it before testing this code on your computer.


How to run for our demo purposes

1) Change the directory path of where alma is on your specific local
computer in the alma\_node.py file

      1.1) You should have cloned the Alma git repository into your local computer already -- remember use the swi branch only

2) Source your ros workspace by : source
~/MyWorkspace/devel/setup.bash

    2.1) You can add this line to your bashrc file to avoid always writing this command

3) Make alma\_node.py executable by doing this command: chmod +x alma_node.py

4) If you want to run alma do this command: ./alma run false keyboard true alfile demo/time1.pl debug 0 /tmp/alma-debug

5) Launch the ros launch file to avoid extra commands of running alma\_node.py by : roslaunch <package name> alma\_launch.launch

	5.1)Remember to change your package name if it not the same as mine (which is alma\_ros\_pkg) 
    
	5.2) This command will run alma_node.py
    
6) To give the alma node some commands here are some examples:

	6.1) To publish to our rostopic alma_node_cmd do any of the
    following commands with example alma commands as input
	
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(q).'
			
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(if(p,q)).'
			
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(p).'
			
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(not     p).'
			
    6.2) To see what is being published in alma_db rostopic do the
    following command
	
            rostopic echo /alma_db
			
    6.3) To load a file
	
    	    rostopic pub -1 /alma_node_cmd std_msgs/String -- 'load("PATH_TO_ALMA/demo/tweet.pl")'

7)  Using in other ROS nodes:

	7.1)  Commands are published to /alma_node_cmd with type string.
    These correspond to commands you would type into an interactive
    alma session (in fact, we are currently running an interactive
    sessions as a subprocess; this is probably not the best way to
    do this but seems to work for now)

    7.2) To get the KB, subscribe to alma_db, which publishes messages
    of type AlmaDB; these are lists of type AlmaFmla.  An AlmaFmla
    consists of the following fields:

    	     string code: this is a unique code for each formula in the
	         	 KB, similar to a Godel code.  In the case of named
	         	 formulae, this will be the name

	     string fmla: the actual formula

	     bool trusted: whether or not this formula is trusted
    	     	  	   (distrust occurs when formulae are involved
    	     	  	   in a contradiction)

8) The functionalilty of carne was twofold: to handle inter-process
   communication and to allow for alma to execute code externally.
   The first function is handled by ROS itself; the second is handled
   by the alma_cnc node.  

	8.1) This node scans the alma database and looks for predicates of
		the form 'action(X)'.  It then removes those predicates,
		executes an appropriate command and writes 'doing(X)' to the
		KB.  Upon completion, 'doing(X)' is replaced with 'done(X)'

9) To do:

	9.1) Interact directly with prolog using toplevel.pl, rather than
   		using an interactive session in a subprocess.  Specifically it
   		would good to interact more directly with the database.

	9.2) Set up a parameter server that allows, among other things,
   	 for the functionality of the alma command-line.

	9.3) Allow for saving KBs.

  

   
