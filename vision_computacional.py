#!/usr/bin/env python3
import serial
import time
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import math
import numpy as np
import cv2
def redondeo(n):
	a = n/5
	b = round(a)
	c = b*5
	return (c)
#Comunicación serial
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
ser.reset_input_buffer()

cap = cv2.VideoCapture(0)
while True:
	if ser.in_waiting > 0:
		linea = ser.readline().decode('utf-8').rstrip()
		#line = int(line)
		print(linea)
		if (linea == "1"):
			print("se esta ejecutando la raspberry: imagen")
			#cap = cv2.imread(f'/home/eduar/Descargas/SoC/Reto/projects/jpg/{i}.jpg')
			ret, frame = cap.read()
			alpha = 1 #contraste
			beta = -100 #brillo
			adjusted = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta) 
			img_grey = cv2.cvtColor(adjusted, cv2.COLOR_BGR2GRAY) #b&w 
			img_rgb = cv2.cvtColor(adjusted, cv2.COLOR_BGR2RGB) 
			img_blur = cv2.GaussianBlur(img_grey,(7,7), 0, 0)
			img_canny = cv2.Canny(img_blur, 50, 120)
			cv2.imwrite("prueba.jpg", img_canny)
			plt.imshow(img_canny, cmap='gray')
			#cv2.imshow('Video', img_canny)
			img_canny.shape
			
			#Selección de la ROI
			vertices = np.array([[(0,450),(0, 100), (400, 100), (400,450)]], dtype=np.int32)	
			img_roi = np.zeros_like(img_grey)
			cv2.fillPoly(img_roi, vertices, 255)
			img_mask = cv2.bitwise_and(img_canny, img_roi)
			plt.imshow(img_mask, cmap='gray')
			cv2.imshow('Video', img_mask)
			
			#HOUGH_TRANSFORMATION
			rho = 2			# resolución de rho en pixeles
			theta = np.pi/180  # resolución de theta en radianes
			threshold = 40	 # mínimo número de votos para ser considerado una línea
			min_line_len = 50  # mínimo número de pixeles para que se forme una línea
			max_line_gap = 10  # máximo espacio en pixeles entre segmentos de línea	
			lines = cv2.HoughLinesP(img_mask, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
			# se crea un fondo negro del tamaño de la imagen con bordes
			img_lines = np.zeros((img_mask.shape[0], img_mask.shape[1], 3), dtype=np.uint8)
			# se dibujan cada una de las líneas sobre la imagen con fondo negro
			cont = 0
			suma = 0
			if lines is not None and lines.any():
				for line in lines:
					for x1,y1,x2,y2 in line:	
						cv2.line(img_lines, (x1, y1), (x2, y2), [255, 0, 0], 30)
						y = y2 - y1
						x = x2 - x1
						if x == 0:
							deg = 90
						else:
							m = y/x
							if m == 0:
								continue
						#print(f"la pendiente es : {m}")
							rad = math.atan(m)
							deg = math.degrees(rad)
				
						if deg < 0:
							deg += 180
						#print(deg)
						suma += deg
						cont += 1
				if cont > 0:
					deg_prom = suma/cont
					
				gamma = 0 # angulo de conduccion
				if (deg_prom == 90):
					gamma = 0
					mov = "000"
				elif (deg_prom < 90): #izquierda
					gamma = 90 - deg_prom
					g = round(gamma)
					if (g < 11):
						mov = "000"
					else:
						mov = "-"+ str(g)
					print(g)
					print(mov)
				else: #derecha
					gamma = deg_prom - 90
					g = round(gamma)
					if (g < 11):
						mov = "000"
					else:
						mov = "+" + str(g)
					print(g)
					print(mov)
			else:
				if mov != "000":
					mov = int(mov)
					if mov < 0:
						mov = "+" + str(g)
					else:
						mov = "-" + str(g)

			#Envia el angulo
			ser.write(str(mov).encode())
			print(f"se envio {mov}")
			
			if cv2.waitKey(1) == 13:
				break

cap.release()
cv2.destroyAllWindows()
