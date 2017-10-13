import xlsxwriter
import serial


#for x in range(1,3):
#	t = input('tensao: ')
#	print x
#	worksheet.write(x, 0, t)
#	ea = input("erro A: ")
#	worksheet.write(x, 1, ea)
#	eb = input("erro B: ")
#	worksheet.write(x, 2, eb)
def ler_serial():
	start = False
	x = 1
	y = 0
	while(True):
		data = arduino.readline()[:-2]
		if data == "$":
			y=0
			data = arduino.readline()[:-2]
			print data
			worksheet.write(x, y, data)
			y+=1
			data = arduino.readline()[:-2]
			print data
			worksheet.write(x, y, data)
			y+=1
			data = arduino.readline()[:-2]
			print data
			worksheet.write(x, y, data)
			x+=1
		elif data == "#":
			print "TERMINA AQUI"
			break;
		
	


#setup
workbook = xlsxwriter.Workbook('Firm_test.xlsx')
worksheet = workbook.add_worksheet()
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)

#worksheet.write(2, 0, 123)
worksheet.write('A1', 'tensao')
worksheet.write('B1', 'motor A')
worksheet.write('C1', 'motor B')
ler_serial()
workbook.close()
