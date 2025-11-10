import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class AnimationPlot:

    def animate(self, i, dataList, ser):
        ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
        arduinoData_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string
        #print(i)                                           # 'i' is a incrementing variable based upon frames = x argument

        try:
            arduinoData_float = float(arduinoData_string)   # Convert to float
            dataList.append(arduinoData_float)              # Add to the list holding the fixed number of points to animate

        except:                                             # Pass if data point is bad                               
            pass

        dataList = dataList[-50:]                           # Fix the list size so that the animation plot 'window' is x number of points
        
        ax.clear()                                          # Clear last data frame
        
        self.getPlotFormat()
        ax.plot(dataList)                                   # Plot new data frame
        

    def getPlotFormat(self):
        ax.set_ylim([0, 3000])                              # Set Y axis limit of plot
        ax.set_title("Arduino Data")                        # Set title of figure
        ax.set_ylabel("Value")                              # Set title of y axis

dataList = []                                           # Create empty list variable for later use
                                                        
fig = plt.figure()                                      
ax = fig.add_subplot(111)                               

realTimePlot = AnimationPlot()

ser = serial.Serial("COM3", 9600)                       
time.sleep(2)                                           

ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(dataList, ser), interval=100) 

plt.show()                                              
ser.close()   