import os 
from os import sys
from data_clct_conf_class import *
from control_unreal import *
from scp import SCPClient
import paramiko
import sys
from shutil import copy 
import time
import traceback

data_clct_conf_file_addr = "../config/stats.json"
def generate_csv(data_clct_conf_file_addr):
	data_clct_conf_obj = DataClctConf(data_clct_conf_file_addr) 
	data = data_clct_conf_obj.get_config_data().values()[0]
	stat_keys = data[0].keys()	
	data_dictionarized = {}
	#stat_file_addr = data_clct_conf_obj.get_config_data()["mav_bench_dir"] + "data/"+ data_clct_conf_obj.get_config_data()["application"]+ "/"+"stats.csv"
	stat_file_addr = "../config/stats.csv" 

	#--- create a dictionary out of the data
	for stats in stat_keys: 
		data_dictionarized[stats] = map(lambda x: x.get(stats), data)

	#--- dump the dictionary in a csv file
	with open(stat_file_addr, 'w') as file:
		for stats in stat_keys:
			file.write(','.join(map(str, [stats] + data_dictionarized[stats])) + "\n")
		

#combine the two jsons and write it into json1
def combine_json(file_list):
	head = []
	result = {}
	with open("result.json", "w") as outfile:
	    for f in file_list:
		with open(f, 'rb') as infile:
		    file_data = json.load(infile)
		    head = head + (file_data.values()[0])
		    main_key = file_data.keys()[0] 
	    
	    result[main_key] = head;
	    json.dump(result, outfile)
	copy("result.json", file_list[0])
		

def main():
	#copy first	
	combine_json(["../config/test_data.json", "../config/test_data2.json"])
	generate_csv("../config/test_data.json")
        #copy back	
	#remove all the temps

if __name__ == "__main__":
	main()

