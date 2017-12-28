import os 
from os import sys
import json 

class data_clct_conf:
    def __init__(self, input_file_addr):
        self.input_file_addr = input_file_addr #input file to parse
        self.set_config_data();

    def set_config_data(self):
        if not(os.path.isfile(self.input_file_addr)):
            print "file:" + self.input_file_addr+ " doesn't exist"
            sys.exit()
        with open(self.input_file_addr) as data_file:
            data = json.loads(data_file.read())
            self.config_data = data

    def get_config_data(self):
        return self.config_data;
