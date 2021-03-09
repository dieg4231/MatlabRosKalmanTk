#!/usr/bin/env python3

import rospkg
from tkinter import * 
#from tkFont import Font
from tkinter import font as Font
import threading
#import ttk
from tkinter import ttk
import time
import math
from PIL import Image,ImageTk
from PIL import ImageDraw
#import tkMessageBox
from tkinter import messagebox as tkMessageBox
import os
import numpy as np
import subprocess
import rospy

from simulator.msg import Parameters
from std_msgs.msg import Float32MultiArray

 
class MobileRobotSimulator(threading.Thread):
	
	def __init__(self):
		
		threading.Thread.__init__(self)
		
		self.stopped = False 
		# map size in meters
		self.mapX = 0 
		self.mapY = 0
		# canvas size in pixels
		self.canvasX= 800
		self.canvasY= 800
		# robot position and angle
		self.robot_theta=0
		self.robotX=-100
		self.robotY=-100

		self.p_giro=0
		self.p_distance=0
		
		self.polygonMap = []
		self.polygons_centorids = []
		self.nodes_image = None
		self.light=-1
		self.robot=-1

		self.flagOnce=False

		self.light_x = 0
		self.light_y = 0
		self.startFlag = False

		self.lasers = []
		self.sensors_value = [None]*512;
		self.sensors_values = [None]*512;
		self.sensors_values_aux = [None]*512;
		self.sensors_values_aux_old = [None]*512;
		
		#for i in range(512):
			#self.sensors_value.append(0)
		#	self.sensors_values.append(0)
			#self.sensors_values_aux.append(0)


		self.graph_list = [200]
		for i in range(200):
			self.graph_list.append(0)

		self.rewind=[]
		self.trace_route= []
		self.varShowNodes   = False
		self.grid =[]
		self.contador = 0;
		self.contador_ = 0;
		self.bandera = True

		self.X_pose = 0
		self.Y_pose = 0

		self.num_polygons = 0  #How many polygons exist in the field.
		self.polygons = []	   #Stors polygons vertexes
		self.polygons_mm = []  #Stors 2 vertexses  for each polygon the maximum and minimum  x and y  points.

		self.objects_data = []
		self.grasp_id = False
		self.current_object = -1;
		self.current_object_name = -1;

		self.initX = 0
		self.initY = 0
		self.initR = 0

		self.map_path = 0

		self.predic_elipse_x = 0
		self.predic_elipse_y = 0
		self.predic_elipses = []
		self.predic_elipse_angle = 0

		self.actual_elipse_x = 0
		self.actual_elipse_y = 0
		self.actual_elipses = []
		self.actual_elipse_angle = 0

		self.start()

	def kill(self):  # When press (x) window
		self.stopped = True
		self.startFlag=False
		time.sleep(2)
		self.root.quit()
		self.root.destroy()

##################################
##################################
# 
#   MAP
#
##################################
##################################
	def prediction_plot(self,*args):
		rospack = rospkg.RosPack()
		image = Image.open(rospack.get_path('simulator')+'/src/gui/predict_elipse.png')
		self.predic_elipses.append( ImageTk.PhotoImage(image.rotate(self.predic_elipse_angle)) )
		#self.w.create_image(  (self.predic_elipse_x*self.canvasX)/self.mapX  , self.canvasY-(self.predic_elipse_y.get()*(self.canvasY/self.mapY))  , image = self.predic_elipses[-1])


	def actualization_plot(self,*args):
		rospack = rospkg.RosPack()
		image = Image.open(rospack.get_path('simulator')+'/src/gui/actual_elipse.png')
		self.actual_elipses.append( ImageTk.PhotoImage(image.rotate(self.actual_elipse_angle)) )
		self.w.create_image( (self.actual_elipse_x*self.canvasX)/self.mapX  , self.canvasY-(self.actual_elipse_y.get()*(self.canvasY/self.mapY))  , image = self.actual_elipses[-1])


	def clear_canvas(self):
		self.w.delete("all")
		self.read_map("nada")


	def print_grid(self,line_per_m = 10):
		for i in self.grid :
			self.w.delete(i)
		self.grid =[]

		for i in range(0, int(self.mapX)*line_per_m):
			self.grid.append(self.w.create_line( i * self.canvasX/(self.mapX*line_per_m),0, i*self.canvasX/(self.mapX*line_per_m), self.canvasY,  dash=(4, 4), fill=self.gridColor))
		for i in range(0, int(self.mapY)*line_per_m):
			self.grid.append(self.w.create_line( 0, i*self.canvasY/(self.mapY*line_per_m),self.canvasX, i*self.canvasY/(self.mapY*line_per_m),   dash=(4, 4), fill=self.gridColor))


	def read_map(self,*args):  # It reads maps from  src/data/[map].wrl folder 
		self.num_polygons=0
		for polygon in self.polygonMap :
			self.w.delete(polygon)
		
		for polygon in self.polygons_centorids :
			self.w.delete(polygon)

		self.polygonMap = []
		self.polygons = []	
		self.polygons_mm = []
		self.polygons_centorids = []
		cta =0;
		

		try:
			#self.w.delete("all")
			map_file = open(self.map_path.get(),'r') #Open file
			lines = map_file.readlines()                          #Split the file in lines
			for line in lines: 									  #To read line by line
				words = line.split()	                          #To separate  words 
				if words:										  #To avoid empty lines							
					if words[0] == "(":							  #To avoid coments
						if words[1] == "dimensions":			  #To get world dimensions
							self.mapX = float (words[3])	
							self.mapY = float (words[4])
							self.print_grid()
						elif words[1] == "polygon":				  #to get polygons vertex

							vertex_x = [ ( ( self.canvasX * float(x) ) / self.mapX ) for x in words[4:len(words)-1:2]	]
							vertex_y = [ ( self.canvasY -  ( self.canvasY * float(y) ) / self.mapY ) for y in words[5:len(words)-1:2]	]
							vertex_y_calculus = [ (( self.canvasY * float(y) ) / self.mapY ) for y in words[5:len(words)-1:2]	]
							
							vx = [ float(x)for x in words[4:len(words)-1:2] ]	
							vy = [ float(y)for y in words[5:len(words)-1:2] ]
						
							vertexs = list( zip( vertex_x , vertex_y) )

						
						

							self.polygons.append( list(zip( vertex_x , vertex_y_calculus)))
							self.polygonMap.append(self.w.create_polygon(vertexs, outline=self.obstaclesOutlineColor, fill=self.obstacleInnerColor, width=1))	
							self.polygons_centorids.append(self.w.create_oval(np.asarray(vertex_x).sum()/len(vertex_x)-1 ,np.asarray(vertex_y).sum()/len(vertex_y)-1,np.asarray(vertex_x).sum()/len(vertex_x)+1 ,np.asarray(vertex_y).sum()/len(vertex_y)+1,fill = '#cb1801'  ) )
							self.polygons_centorids.append(self.w.create_text(np.asarray(vertex_x).sum()/len(vertex_x)-1 ,np.asarray(vertex_y).sum()/len(vertex_y)-10, text=str(cta)) )
							cta = cta+1
							#self.w.create_text( self.canvasX * float(words[4]) / self.mapX,  self.canvasY -  ( self.canvasY * float(words[5]) ) / self.mapY, text=str(pp))
							max_x = 0;
							max_y = 0;
							min_x = 999;
							min_y = 999;

							for i in vertexs:
								if max_x < i[0]:
									max_x = i[0]
								if min_x > i[0]:
									min_x = i[0]

							for i in vertexs:
								if max_y < i[1]:
									max_y = i[1]
								if max_y > i[1]:
									min_y = i[1]
							self.polygons_mm.append( [[max_x,max_y],[min_x,min_y] ] )
					
							self.num_polygons = self.num_polygons+1
			for p in self.polygons:
				p.append(p[0])
		except IOError:
			pass


##################################################
##################################################
#
# ROS
#
##################################################
##################################################

	def gui_init(self):

		self.backgroundColor = '#EDEDED';#"#FCFCFC";
		self.entrybackgroudColor = "#FBFBFB";##1A3A6D";
		self.entryforegroundColor = '#37363A';
		self.titlesColor = "#303133"
		self.menuColor = "#ECECEC"
		self.menuButonColor = "#375ACC"
		self.menuButonFontColor = "#FFFFFF"
		self.obstacleInnerColor = '#447CFF'
		self.obstaclesOutlineColor="#216E7D"#'#002B7A'
		self.buttonColor = "#1373E6"
		self.buttonFontColor = "#FFFFFF"
		self.canvasColor = "#FFFFFF"
		self.gridColor = "#D1D2D4"
		self.wheelColor  = '#404000'  
		self.robotColor  = '#F7CE3F'  
		self.hokuyoColor = '#4F58DB' 
		self.arrowColor  = '#1AAB4A' 
		self.laserColor  = "#00DD41" 


		
		self.root = Tk()
		self.root.protocol("WM_DELETE_WINDOW", self.kill)
		self.root.title("Kalman State Visualization")
		
	
		self.content   = Frame(self.root)
		self.frame     = Frame(self.content,borderwidth = 5, relief = "flat", width = 600, height = 900 ,background = self.backgroundColor)
		self.rightMenu = Frame(self.content,borderwidth = 5, relief = "flat", width = 300, height = 900 ,background = self.backgroundColor)
		self.w = Canvas(self.frame, width = self.canvasX, height = self.canvasY, bg=self.canvasColor)
		self.w.pack()
		
		self.headLineFont = Font.Font( family = 'Helvetica' ,size = 12, weight = 'bold')
		self.lineFont     = Font.Font( family = 'Helvetica' ,size = 10, weight = 'bold')
		self.buttonFont   = Font.Font( family = 'Helvetica' ,size = 8, weight = 'bold')


		self.lableEnvironment   = Label(self.rightMenu ,text = "Settings"     ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.labelFile          = Label(self.rightMenu ,text = "Environment:"           ,background = self.backgroundColor ,font = self.lineFont)
		self.labelSteps         = Label(self.rightMenu ,text = "Steps:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelBehavior		= Label(self.rightMenu ,text = "Behavior:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelLightX        = Label(self.rightMenu ,text = "Light X:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelLightY        = Label(self.rightMenu ,text = "Light Y:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelStepsExcec        = Label(self.rightMenu ,text = "Steps:"          ,background = self.backgroundColor ,font = self.lineFont)
		self.labelConfiguration = Label(self.rightMenu ,text = "Configurations:" ,background = self.backgroundColor ,font = self.lineFont)
			
		##### Rigth menu widgets declaration

		# Environment

		self.varFaster    = IntVar()
		self.varShowSensors = IntVar()
		self.varAddNoise    = IntVar()
		self.varLoadObjects    = IntVar()
		

		self.checkFaster    = Checkbutton(self.rightMenu ,text = 'Fast Mode'    ,variable = self.varFaster    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkShowSensors = Checkbutton(self.rightMenu ,text = 'Show Sensors' ,variable = self.varShowSensors ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkAddNoise    = Checkbutton(self.rightMenu ,text = 'Add Noise'    ,variable = self.varAddNoise    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		self.checkLoadObjects    = Checkbutton(self.rightMenu ,text = 'Load Objects'    ,variable = self.varLoadObjects    ,onvalue = 1 ,offvalue = 0 ,background = self.backgroundColor)
		

		self.checkFaster      .deselect()
		self.checkShowSensors .select()
		self.checkAddNoise    .deselect()
		self.checkLoadObjects    .deselect()
	
		# Robot 

		self.lableRobot     = Label(self.rightMenu ,text = "Robot"              ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont )
		self.labelPoseX     = Label(self.rightMenu ,text = "Pose X:"            ,background = self.backgroundColor ,font = self.lineFont)
		self.labelPoseY     = Label(self.rightMenu ,text = "Pose Y:"            ,background = self.backgroundColor ,font = self.lineFont)
		self.labelAngle     = Label(self.rightMenu ,text = "Angle:"             ,background = self.backgroundColor ,font = self.lineFont)
		

		# buttons

		self.lableSimulator      = Label (self.rightMenu ,text = "Simulator" ,background = self.backgroundColor ,foreground = self.titlesColor ,font = self.headLineFont)
		self.buttonLastSimulation= Button(self.rightMenu ,width = 20, text = "Run last simulation" ,state="disabled", foreground = self.buttonFontColor ,background = self.buttonColor , font = self.buttonFont )
		self.buttonRunSimulation = Button(self.rightMenu ,width = 20, text = "Run simulation", foreground = self.buttonFontColor ,background = self.buttonColor,font = self.buttonFont )
		self.buttonStop          = Button(self.rightMenu ,width = 20, text = "Stop", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont,   command = self.clear_canvas)


		#### Right menu widgets grid			

		# Environment
		
		
		# Robot

		self.lableRobot     .grid(column = 4 ,row = 0 ,sticky = (N, W) ,padx = (5,5))     
		self.labelPoseX     .grid(column = 4 ,row = 1 ,sticky = (N, W) ,padx = (10,5))
		self.labelPoseY     .grid(column = 4 ,row = 2 ,sticky = (N, W) ,padx = (10,5))
		self.labelAngle     .grid(column = 4 ,row = 3 ,sticky = (N, W) ,padx = (10,5))

			
		# Sensors

		# buttons

		self.lableSimulator     .grid(column = 4 ,row = 12  ,sticky = (N, W) ,padx = (5,5))
		self.buttonRunSimulation.grid(column = 4 ,row = 17 ,sticky = (N, W) ,padx = (10,5))
		self.buttonStop         .grid(column = 4 ,row = 18 ,sticky = (N, W) ,padx = (10,5))

		self.content  .grid(column = 0 ,row = 0 ,sticky = (N, S, E, W))
		self.frame    .grid(column = 0 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))
		self.rightMenu.grid(column = 3 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))

		self.root.columnconfigure(0, weight=1)
		self.root.rowconfigure(0, weight=1)
		self.content.columnconfigure(0, weight = 3)
		self.content.columnconfigure(1, weight = 3)
		self.content.columnconfigure(2, weight = 3)
		self.content.columnconfigure(3, weight = 1)
		self.content.columnconfigure(4, weight = 1)
		self.content.rowconfigure(1, weight = 1)

		self.map_path = StringVar(value=" ")
		self.map_path.trace("w", self.read_map)

		self.predic_elipse_y = DoubleVar(value=0)
		self.predic_elipse_y.trace("w", self.prediction_plot)

		self.actual_elipse_y = DoubleVar(value=0)
		self.actual_elipse_y.trace("w", self.actualization_plot)

	def run(self):	
		self.gui_init()
		self.read_map()
		self.root.mainloop()



map_old = " "

prediction_x_old = 70969078;
actualization_x_old = 70969078;
simul = MobileRobotSimulator()



def predictionCallback(data):
	global prediction_x_old
	if prediction_x_old != data.data[0]:
		rospack = rospkg.RosPack()
		print(data.data)
		prediction_x_old = data.data[0]
		
		canvasX = simul.canvasX
		canvasY = simul.canvasY
		map_x = simul.mapX
		map_y = simul.mapY
		#image = Image.new('RGBA', ( int(data.data[3] * 100000000) , int( data.data[4] * 100000000 ) ) )
		image = Image.new('RGBA', (canvasX , canvasY) ) 
		draw = ImageDraw.Draw(image)
		draw.ellipse( (  (canvasX/2)-1 , (canvasY/2)-1 , (canvasX/2)+1 , (canvasY/2)+1 ), fill= '#994FDB')
		draw.ellipse( (  (canvasX/2)-(int( data.data[3] * canvasX/map_x )/2) , (canvasY/2)-(int( data.data[4] * canvasY/map_y )/2) , (canvasX/2)+(int(data.data[3] * canvasX/map_x)/2) , (canvasY/2)+(int( data.data[4] * canvasY/map_y )/2) ), outline = '#9C4FDB')
		image.save(rospack.get_path('simulator')+'/src/gui/predict_elipse.png')
		simul.predic_elipse_angle = data.data[2]*(180/math.pi) 
		simul.predic_elipse_x = data.data[0]
		simul.predic_elipse_y.set( data.data[1] )

def actualizationCallback(data):
	global actualization_x_old
	if actualization_x_old != data.data[0]:
		rospack = rospkg.RosPack()
		print(data.data)
		actualization_x_old = data.data[0]
		canvasX = simul.canvasX
		canvasY = simul.canvasY
		map_x = simul.mapX
		map_y = simul.mapY
		#image = Image.new('RGBA', ( int(data.data[3] * 100000000) , int( data.data[4] * 100000000 ) ) )
		image = Image.new('RGBA', (canvasX , canvasY) ) 
		draw = ImageDraw.Draw(image)
		draw.ellipse( (  (canvasX/2)-1 , (canvasY/2)-1 , (canvasX/2)+1 , (canvasY/2)+1 ), fill= '#33DB17')
		draw.ellipse( (  (canvasX/2)-(int( data.data[3] * canvasX/map_x )/2) , (canvasY/2)-(int( data.data[4] * canvasY/map_y )/2) , (canvasX/2)+(int(data.data[3] * canvasX/map_x)/2) , (canvasY/2)+(int( data.data[4] * canvasY/map_y )/2) ), outline = '#33DB17')
		image.save(rospack.get_path('simulator')+'/src/gui/actual_elipse.png')
		
		simul.actual_elipse_angle = data.data[2]*(180/math.pi) 
		simul.actual_elipse_x = data.data[0]
		simul.actual_elipse_y.set( data.data[1] )



def paramsCallback(data):
	global map_old
	rospack = rospkg.RosPack()
	if map_old != data.world_name:
		simul.map_path.set(rospack.get_path('simulator')+'/src/data/'+data.world_name+'/'+data.world_name+'.wrl');
		map_old = data.world_name;
		

if __name__ == "__main__":
	rospy.init_node('kalman_vizualization_node')
	rospy.Subscriber("simulator_parameters_pub", Parameters, paramsCallback)
	rospy.Subscriber("/blob_prediction", Float32MultiArray ,  predictionCallback)
	rospy.Subscriber("/blob_actualization", Float32MultiArray ,  actualizationCallback)
	#time.sleep(5)
	

	rospy.spin()
