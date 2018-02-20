#import spur
import os 
import win32gui, win32con
import time
from os import sys
from data_clct_conf_class import *
from control_unreal import *
import traceback
import signal
from scp import SCPClient
import paramiko
import sys
from shutil import copy 
import time

import argparse
parser = argparse.ArgumentParser(description='DARwing collect data.')
parser.add_argument('--config', metavar='c', type=str,
                    default="..\config\data_clct_conf.json",
                    help='config json file path')

args = parser.parse_args()
data_clct_conf_file_addr = args.config


def creat_ssh_client(data_clct_conf_obj):
    
    # paramiko
    ssh_client=paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(data_clct_conf_obj.get_config_data()["host_to_cnct_to"],
            22,
            data_clct_conf_obj.get_config_data()["usr_name"],
            data_clct_conf_obj.get_config_data()["pass_code"])
    return ssh_client 

def start_unreal(data_clct_conf_obj):
    game_path =  data_clct_conf_obj.get_config_data()["game_path"]
    if not(os.path.isfile(game_path)):
        print("file:" + game_path + " doesn't exist")
        sys.exit()
    
    start_game(game_path); 
    return

   
def get_ros_cmd(data_clct_conf_obj):
    application = data_clct_conf_obj.get_config_data()["application"]
    
    if (application == "package_delivery"):
        return "roslaunch package_delivery package_delivery.launch"
        #return "roslaunch package_delivery y.launch"
    elif (application == "scanning"):
        return "roslaunch package_delivery scanning.launch"
    elif (application == "mapping"):
        return "roslaunch mapping_and_sar mapping.launch"
    elif (application == "sar"):
        return "roslaunch mapping_and_sar sar.launch"
    elif (application == "follow_the_leader"):
        return "roslaunch follow_the_leader follow_the_leader.launch"
    else:
        print("this application not defined")
        sys.exit()

def get_supervisor_cmd(data_clct_conf_obj):
   mav_bench_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   termination =  data_clct_conf_obj.get_config_data()["termination"]

   return  "python "+\
            mav_bench_dir+"run_time/supervisor.py" +\
            " " + mav_bench_dir  +\
            " " + str(termination["time_based"]) + \
            " " +  str(termination["time_to_terminate"] )


def get_pre_mission_cmd():
    return "./catkin_ws/src/mav-bench/misc/pre_mission_cmds.sh"

def schedule_tasks(data_clct_conf_obj, ssh_client):
    
    #--- cmds to schedul e
    src_ros_cmd = "source " + data_clct_conf_obj.get_config_data()["catkin_dir"]+"/devel/setup.bash"
    ros_launch_cmd = get_ros_cmd(data_clct_conf_obj) 
    run_time_supervisor_cmd = get_supervisor_cmd(data_clct_conf_obj)
    pre_mission_cmds = get_pre_mission_cmd()
    all_cmds = src_ros_cmd + ";" + run_time_supervisor_cmd + "& " +  pre_mission_cmds +  "|" + ros_launch_cmd 
    #--- pramiko
    stdin,stdout,stderr= ssh_client.exec_command(all_cmds, get_pty=True)
    outlines = stdout.readlines() 
    result=''.join(outlines)
    print(result)
    # errlines = stderr.readlines() 
    # resp_err=''.join(errlines)
    # print(resp_err)
    return result

def copy_results_over(data_clct_conf_obj, ssh_client):
   mav_bench_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   application = data_clct_conf_obj.get_config_data()["application"]
   stats_file_name_on_comp_computer = data_clct_conf_obj.get_config_data()["stats_file_on_comp_computer"]
   stats_dir_on_host = data_clct_conf_obj.get_config_data()["stats_dir_on_host"]
   data_addr = mav_bench_dir +"/data/"+application+"/" + stats_file_name_on_comp_computer
   
   scp_client = SCPClient(ssh_client.get_transport()) 
   scp_client.get(data_addr)
   copy(stats_file_name_on_comp_computer, stats_dir_on_host);  

def restart_unreal():
    restart_level();

def stop_unreal():
    stop_game();

def parse_results(result):
    return 

def minimize_the_window():
    time.sleep(5);
    Minimize = win32gui.GetForegroundWindow()
    win32gui.ShowWindow(Minimize, win32con.SW_MINIMIZE) 

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
	stop_unreal()
	sys.exit(0)

def write_to_stats_file(stat_file_addr, string_to_write, data_clct_conf_obj, ssh_client):
        python_file_to_run = data_clct_conf_obj.get_config_data()["mav_bench_dir"]+ "common/python_files/write_to_file.py"
        cmd = "python" + " " + python_file_to_run + " " + stat_file_addr + " " + string_to_write
        stdin,stdout,stderr= ssh_client.exec_command(cmd, get_pty=True)
        outlines = stdout.readlines() 
        result=''.join(outlines)
        print(result)
        # errlines = stderr.readlines() 
        # resp_err=''.join(errlines)
        # print(resp_err)
        return result


def main():
    try:
        data_clct_conf_obj = DataClctConf(data_clct_conf_file_addr) #parse config file and instantiate a 
        num_of_runs = data_clct_conf_obj.get_config_data()["number_of_runs"]
        #start_unreal(data_clct_conf_obj)
	ssh_client = creat_ssh_client(data_clct_conf_obj)     
        stat_file_addr = data_clct_conf_obj.get_config_data()["mav_bench_dir"]+ "data/"+ data_clct_conf_obj.get_config_data()["application"]+"/"+"stats.json"
        application = data_clct_conf_obj.get_config_data()["application"]
        
        #minimize_the_window()
        write_to_stats_file(stat_file_addr, '{"experiment_set":[',  data_clct_conf_obj, ssh_client)
        #-- start collecting data 
	for  run_ctr  in range(0, num_of_runs):
            result = schedule_tasks(data_clct_conf_obj, ssh_client)
            #copy_results_over(data_clct_conf_obj, ssh_client);
            #parse_results(result)
	    #time.sleep(5) 
            restart_unreal()
	    #time.sleep(5) 
            restart_unreal()
            write_to_stats_file(stat_file_addr, '"app":'+str(application)+",",  data_clct_conf_obj, ssh_client)
            if (run_ctr < num_of_runs - 1): 
                write_to_stats_file(stat_file_addr, '"experiment":'+str(run_ctr)+"},",  data_clct_conf_obj, ssh_client)
        stop_unreal() 
        
        
        write_to_stats_file(stat_file_addr, '"experiment":'+str(run_ctr)+"}",  data_clct_conf_obj, ssh_client)
        write_to_stats_file(stat_file_addr, "]}",  data_clct_conf_obj, ssh_client)
    except Exception as e:
	pass
    	print(traceback.format_exception(*sys.exc_info()))
	#stop_unreal()

if __name__ == "__main__":
    main()

