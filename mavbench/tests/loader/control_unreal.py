import os
from subprocess import call, Popen

def messages_dir():
	return os.path.expanduser("~/Documents/AirSim/ok");

def start_game(path, in_editor=False):
	if in_editor:
		print("Not impolemented yet")
	else:
		Popen(path)

def stop_game():
	f = open(os.path.join(messages_dir(), "exit"), "w");
	f.close()
	
def change_level(level):
	path = os.path.join(messages_dir(), "change_level.txt")
	f = open(path, "w");
	f.write(level)
	f.close()
	os.rename(path, os.path.join(messages_dir(), "change_level"))
	
def restart_level():
	f = open(os.path.join(messages_dir(), "restart"), "w");
	f.close()
