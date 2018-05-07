#!/usr/bin/env python


import sys
import rospy
import json
import Tkinter as tk
from std_msgs.msg import String

action_dictionary = {
	"move_to" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Target"]
	},
	"open_door" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Door Name"]
	},
	"detect" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Object"]
	},
	"pickup" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Object"]
	},
	"set_down" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Object", "Preposition", "Target"]
	},
	"unlock_door" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Door"]
	},
	"turn_on_lights" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Lights"]
	},
	"navigate_to" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Target"]
	},
	"fetch" : {
		"failure_conditions" : {

		},
		"parameterization" : ["Object", "Location", "Preposition", "Target"]
	}
}
sub = None
pub = None



class UserWindow():

	def __init__(self):
		self.master = tk.Tk()
		self.master.title("Robot Control Input")
		self.master.geometry("600x400")
		self.action_choice = None
		self.instruction_label = None
		self.command = None
		self.choice_widgets = []
		self.parameter_entrys = []
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
		for parameter in action_dictionary[self.command]['parameterization']:
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
		command = {}
		command[self.command] = {}
		command[self.command]["failure_conditions"] = action_dictionary[self.command]["failure_conditions"]
		command[self.command]["parameterization"] = []
		for e in self.parameter_entrys:
			command[self.command]["parameterization"].append(e.get())
		pub.publish(json.dumps(command))
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
	sub = rospy.Subscriber("/dsi/gui_startup", String, startGui)
	pub = rospy.Publisher("/dsi/human_command", String, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	main()
