﻿Create a link:

	your github folder with the arduino libraries -> the arduino libraries on your PC


On Windows:

	mklink /J "<Biblioteca do Arduíno>" "<Pasta do github>"


	Example:

		mklink /J "C:\Program Files (x86)\Arduino\libraries\headers" "C:\Users\motai\Documents\ieee-very-small\firmware\headers"


Reference: https://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/


How to install new libraries on Linux:

	the header must be placed on ~/arduino/libraries/<header name>/

	example:
		to install de header control.h you must, on your arduino
		folder: 
			open the folder libraries;
			create a folder named control;
			paste the file control.h inside the control folder.
		
