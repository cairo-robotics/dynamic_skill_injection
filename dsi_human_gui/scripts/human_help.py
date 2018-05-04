#!/usr/bin/env python


import sys
import rospy
import Tkinter as tk
from std_msgs.msg import String

action_dictionary = {
	"get_tgt_room" : ["Target"], 
	"find_path" : ["Start Room", "Finish Room"], 
	"determine_door" : ["Room 1", "Room 2"], 
	"move_to" : ["Target"], 
	"open_door" : ["Door Name"], 
	"detect" : ["Object"], 
	"pickup" : ["Object"], 
	"set_down" : ["Object", "Preposition", "Target"], 
	"unlock_door" : ["Door"],
	"turn_on_lights" : ["Lights"], 
	"navigate_to" : ["Target"], 
	"fetch" : ["Object", "Location", "Preposition", "Target"]
}
sub = None
pub = None



class UserWindow():
	master = None
	action_choice = None
	instruction_label = None
	command = None
	choice_widgets = []
	parameter_entrys = []


	def __init__(self):
		self.master = tk.Tk()
		self.master.title("Robot Control Input")
		self.master.geometry("600x400")
		frame = self.createFrame()
		self.createDropdown()
		self.master.mainloop()

	def createDropdown(self):
		self.action_choice = tk.StringVar(self.master)
		self.action_choice.set(list(action_dictionary.keys())[0])
		combo = tk.OptionMenu(self.master, self.action_choice, *action_dictionary.keys())
		combo.grid(column=2, row=4)
		self.choice_widgets.append(combo)
		button1 = tk.Button(self.master, text="Submit Action Choice", command=self.createParameterOptions)
		button1.grid(row=5, column=3) 
		self.choice_widgets.append(button1)

	def createParameterOptions(self):
		self.command = self.action_choice.get()
		for widget in self.choice_widgets:
			widget.destroy()
		self.instruction_label.configure(text="Specify parameters for command '"+self.command+"':")
		row_counter = 2
		self.parameter_entrys=[]
		for parameter in action_dictionary[self.command]:
			newLabel = tk.Label(self.master)
			newLabel.grid(column=1, row=row_counter)
			newLabel.configure(text=parameter)
			newEntry = tk.Entry(self.master)
			newEntry.grid(column=3, row=row_counter)
			self.parameter_entrys.append(newEntry)
			row_counter += 1
		newButton = tk.Button(self.master, text="Submit Command", command=self.submitFinal)
		newButton.grid(column=3, row=row_counter)

	def createFrame(self):
		mainframe = tk.Frame(self.master)
		mainframe.grid(column=4, row=0)
		for col in range(0, 10):
			self.master.grid_columnconfigure(col, minsize=40, pad=0)
		self.master.grid_rowconfigure(0, pad=50)
		self.master.grid_rowconfigure(1, pad=50)
		self.instruction_label = tk.Label(self.master)
		self.instruction_label.grid(column=3, row=1)
		self.instruction_label.configure(text="Select a command to send to the robot:")
		return mainframe

	def submitFinal(self):
		strCommand = self.command+":["
		for e in self.parameter_entrys:
			strCommand = strCommand+e.get()+","
		strCommand = strCommand[:len(strCommand)-1]
		strCommand = strCommand+"]"
		pub.publish(strCommand)
		self.close()

	def close(self):
		self.master.destroy()


def startGui(msg):
	if(msg.data == "run"):
		userWindow = UserWindow()

def main():
	global sub
	global pub
	rospy.init_node("Human_Helper")
	sub = rospy.Subscriber("/gui_startup", String, startGui)
	pub = rospy.Publisher("/human_command", String, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	main()
