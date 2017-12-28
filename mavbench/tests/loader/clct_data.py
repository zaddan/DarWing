import spur
import os 
import win32gui, win32con
import time
from os import sys
from data_clct_conf_class import *
from control_unreal import *
import traceback
import signal

import paramiko
import sys

data_clct_conf_file_addr = "..\configs\data_clct_conf.json"
game_path = "" #C:\Users\Behzad\Desktop\Airsim_game\Test\WindowsNoEditor\MyProject.exe"
shell = "" 
ssh_hndl = ""


def ssh(data_clct_conf_obj):
    global shell
    global ssh_hndl
    """ 
    shell = spur.SshShell(hostname=data_clct_conf_obj.get_config_data()["host_to_cnct_to"],
            username=data_clct_conf_obj.get_config_data()["usr_name"],
            password=data_clct_conf_obj.get_config_data()["pass_code"],
            missing_host_key=spur.ssh.MissingHostKey.accept)
    """ 
     
    # paramiko
    ssh_hndl=paramiko.SSHClient()
    ssh_hndl.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_hndl.connect(data_clct_conf_obj.get_config_data()["host_to_cnct_to"],
            22,
            data_clct_conf_obj.get_config_data()["usr_name"],
            data_clct_conf_obj.get_config_data()["pass_code"])
    


def start_unreal(data_clct_conf_obj):
    global game_path 
    game_path =  data_clct_conf_obj.get_config_data()["game_path"]
    if not(os.path.isfile(game_path)):
        print "file:" + game_path + " doesn't exist"
        sys.exit()
    
    start_game(game_path); 
    return

   
def get_ros_cmd(data_clct_conf_obj):
    application = data_clct_conf_obj.get_config_data()["application"]
    
    if (application == "package_delivery"):
        return "roslaunch package_delivery package_delivery.launch"
    elif (application == "scanning"):
        return "roslaunch package_delivery scanning.launch"
    elif (application == "mapping"):
        return "roslaunch mapping_and_sar mapping.launch"
    elif (application == "sar"):
        return "roslaunch mapping_and_sar sar.launch"
    elif (application == "follow_the_leader"):
        return "roslaunch follow_the_leader follow_the_leader.launch"
    else:
        print "this application not defined"
        sys.exit()


def get_supervisor_cmd(data_clct_conf_obj):
   mav_bench_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   termination =  data_clct_conf_obj.get_config_data()["termination"]

   return  "python "+\
            mav_bench_dir+"run_time/supervisor.py" +\
            " " + mav_bench_dir  +\
            " " + str(termination["time_based"]) + \
            " " +  str(termination["time_to_terminate"] )


def schedule_tasks(data_clct_conf_obj):
    global shell 
    global ssh_hndl
   
 
    #-------- 
    #--- cmds to schedul 
    #-------- 
    #command =  ["./catkin_ws/src/mav-bench/misc/pre_mission_cmds.sh", "|","roslaunch","package_delivery", "package_delivery"]
    ros_launch_cmd = get_ros_cmd(data_clct_conf_obj) 
    run_time_supervisor_cmd = get_supervisor_cmd(data_clct_conf_obj)
    all_cmds = ros_launch_cmd + "|" + run_time_supervisor_cmd;
    #-------- 
    #--- schedule commands 
    #-------- 
    #--- spur
    #result = shell.run(ros_launch_cmd, allow_error=True)
    #print result.output
   
    #--- pramiko
    stdin,stdout,stderr=ssh_hndl.exec_command(all_cmds, get_pty=True)
    outlines = stdout.readlines() 
    result=''.join(outlines)
    print(result)
    # errlines = stderr.readlines() 
    # resp_err=''.join(errlines)
    # print(resp_err)
    
    return result


def restart_unreal():
    restart_level();

def stop_unreal():
    stop_game();


def parse_results(result):
    return 


def minimize_the_window():
    time.sleep(2);
    Minimize = win32gui.GetForegroundWindow()
    win32gui.ShowWindow(Minimize, win32con.SW_MINIMIZE) 

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
	stop_unreal()
	sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    try:
	data_clct_conf_obj = data_clct_conf(data_clct_conf_file_addr) #parse config file and instantiate a 
	start_unreal(data_clct_conf_obj)
	minimize_the_window()
	#data_clct_conf object
	ssh(data_clct_conf_obj)     

	#-- start collecting data 
	for  _ in range(0,data_clct_conf_obj.get_config_data()["number_of_runs"]):
		result = schedule_tasks(data_clct_conf_obj)
		parse_results(result)
		restart_unreal()
		stop_unreal() 
	#print result.return_code
	#print result.output
    except Exception as e:
    	print traceback.format_exception(*sys.exc_info())
	stop_unreal()

if __name__ == "__main__":
    main()

