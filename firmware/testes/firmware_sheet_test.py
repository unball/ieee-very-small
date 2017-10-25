import xlsxwriter
import serial
import plotly
import plotly.graph_objs as go


#for x in range(1,3):
#	t = input('tensao: ')
#	print x
#	worksheet.write(x, 0, t)
#	ea = input("erro A: ")
#	worksheet.write(x, 1, ea)
#	eb = input("erro B: ")
#	worksheet.write(x, 2, eb)
def ler_serial():
	array = [[i*0 for i in range(3000)],[i*0 for i in range(3000)],[i*0 for i in range(3000)]]
	start = False
	x = 1
	y = 0
	while(True):
		data = arduino.readline()[:-2]
		if data == "$":
			y=0
			data = arduino.readline()[:-2]
			array[y][x-1] = data
			print data
			worksheet.write(x, y, data)
			y+=1
			data = arduino.readline()[:-2]
			array[y][x-1] = data
			print data
			worksheet.write(x, y, data)
			y+=1
			data = arduino.readline()[:-2]
			array[y][x-1] = data
			print data
			worksheet.write(x, y, data)
			x+=1
		elif data == "#":
			print "TERMINOU AQUI"
			trace0 = go.Scatter(y=array[0], name='input')
			trace1 = go.Scatter(y=array[1], name='Motor A')
			trace2 = go.Scatter(y=array[2], name='Motor B')
			graph = [trace0, trace1, trace2]
			plotly.offline.plot(graph, filename='my-graph.html')
			break;
		
	


#setup
workbook = xlsxwriter.Workbook('Firm_test.xlsx')
worksheet = workbook.add_worksheet()
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)

#worksheet.write(2, 0, 123)
worksheet.write('A1', 'input')
worksheet.write('B1', 'motor A')
worksheet.write('C1', 'motor B')
ler_serial()
workbook.close()
