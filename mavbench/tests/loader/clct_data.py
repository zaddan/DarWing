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


def creat_ssh_client(user_setting):
    
    # paramiko
    ssh_client=paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(user_setting["host_to_cnct_to"],
            22,
            user_setting["usr_name"],
            user_setting["pass_code"])
    return ssh_client 

def start_unreal(user_setting):
    game_path =  user_setting["game_path"]
    if not(os.path.isfile(game_path)):
        print("file:" + game_path + " doesn't exist")
        sys.exit()
    
    start_game(game_path); 
    return

   
def get_ros_cmd(experiment_setting):
    application = experiment_setting["application"]
    
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

def get_supervisor_cmd(user_setting, experiment_setting):
   mav_bench_dir = user_setting["mav_bench_dir"]
   termination =  experiment_setting["max_run_time"]
   app = experiment_setting["application"]
   return  "python "+\
            mav_bench_dir+"run_time/supervisor.py" +\
            " " + mav_bench_dir  +\
            " " + app +\
            " " +  str(termination)


def get_pre_mission_cmd():
    return "./catkin_ws/src/mav-bench/misc/pre_mission_cmds.sh"

def schedule_tasks(user_setting, experiment_setting, ssh_client):
    
    #--- cmds to schedul e
    src_ros_cmd = "source " + user_setting["catkin_dir"]+"/devel/setup.bash"
    ros_launch_cmd = get_ros_cmd(experiment_setting) 
    run_time_supervisor_cmd = get_supervisor_cmd(user_setting, experiment_setting)
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
"""
def copy_results_over(data_clct_conf_obj, ssh_client):
   mav_bench_dir = data_clct_conf_obj.get_config_data()["mav_bench_dir"]
   application = data_clct_conf_obj.get_config_data()["application"]
   stats_file_name_on_comp_computer = data_clct_conf_obj.get_config_data()["stats_file_on_comp_computer"]
   stats_dir_on_host = data_clct_conf_obj.get_config_data()["stats_dir_on_host"]
   data_addr = mav_bench_dir +"/data/"+application+"/" + stats_file_name_on_comp_computer
   
   scp_client = SCPClient(ssh_client.get_transport()) 
   scp_client.get(data_addr)
   copy(stats_file_name_on_comp_computer, stats_dir_on_host);  
"""

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

def write_to_stats_file(stat_file_addr, string_to_write, user_setting, ssh_client):
        python_file_to_run = user_setting["mav_bench_dir"]+ "common/python_files/write_to_file.py"
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
        user_setting =  data_clct_conf_obj.get_config_data()["user_setting"]
        experiment_setting_list =  data_clct_conf_obj.get_config_data()["experiment_setting_list"]
        total_run_ctr = 0
        for  experiment_setting in  experiment_setting_list:
            num_of_runs = experiment_setting["number_of_runs"]
            application = experiment_setting["application"]
            #start_unreal(user_setting)
            ssh_client = creat_ssh_client(user_setting)     
            stat_file_addr = user_setting["mav_bench_dir"]+ "data/"+ application+"/"+"stats.json"
            write_to_stats_file(stat_file_addr, '{\\"experiments\\":[',  user_setting, ssh_client)
            #minimize_the_window()
            #-- start collecting data 
            for  experiment_run_ctr  in range(0, num_of_runs):
                total_run_ctr += experiment_run_ctr 
                result = schedule_tasks(user_setting, experiment_setting, ssh_client)
                #copy_results_over(user_setting, ssh_client);
                #parse_results(result)
                #time.sleep(5) 
                restart_unreal()
                #time.sleep(5) 
                restart_unreal()
                write_to_stats_file(stat_file_addr, '\\"app\\":\\"'+str(application)+'\\",',  user_setting, ssh_client)
                if (experiment_run_ctr < num_of_runs - 1): 
                    write_to_stats_file(stat_file_addr, '\\"experiment_number\\":'+str(experiment_run_ctr)+"},",  user_setting, ssh_client)
        
        stop_unreal() 
        write_to_stats_file(stat_file_addr, '\\"experiment_number\\":'+str(experiment_run_ctr)+"}",  user_setting, ssh_client)
        write_to_stats_file(stat_file_addr, "]}",  user_setting, ssh_client)
    except Exception as e:
        pass
        print(traceback.format_exception(*sys.exc_info()))
        #stop_unreal()

if __name__ == "__main__":
    main()

