import time, cv2
from djitellopy import Tello

class Tracker:
	"""
	A basic color tracker, it will look for colors in a range and
	create an x and y offset valuefrom the midpoint
	"""

	def __init__(self, height, width, color_lower, color_upper):
		self.color_lower = color_lower
		self.color_upper = color_upper
		self.midx = int(width / 2)
		self.midy = int(height / 2)
		self.xoffset = 0
		self.yoffset = 0
		self.radius = 0

	def draw_arrows(self, frame):
		"""Show the direction vector output in the cv2 window"""
		#cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
		cv2.arrowedLine(frame, (self.midx, self.midy),
						(self.midx + self.xoffset, self.midy - self.yoffset),
						(0, 0, 255), 5)
			
		return frame

	def track(self, frame):
		"""Simple HSV color space tracking"""
		# resize the frame, blur it, and convert it to the HSV
		# color space
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
								cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0]
		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid

			#initialize temp variables
			lowest_error = None 
			best_xoffset = 0
			best_yoffset = 0
			best_x = 0
			best_y = 0
			best_radius = 0
			best_center = (0,0)

			for c in cnts: #iterate through every contour
				#c = max(cnts, key=cv2.contourArea)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				# only proceed if the radius meets a minimum size
				if radius > 10:
					

					temp_xoffset = int(center[0] - self.midx) #store the xoffset and yoffset for each iteration of the loop
					temp_yoffset = int(self.midy - center[1])

					#calculate the distance from the previous x and y by finding the squared error
					sqrd_error = ((temp_xoffset-self.xoffset)**2) + ((temp_yoffset - self.yoffset)**2)

					#Set xoffset and yoffset for the lowest possible distance
					if lowest_error is None or sqrd_error < lowest_error:
						lowest_error = sqrd_error
						best_xoffset = temp_xoffset
						best_yoffset = temp_yoffset
						best_x = x
						best_y = y
						best_radius = radius
						best_center = center


			#Set xoffset and yoffset to the best values calculated in the for loop		
			self.xoffset = best_xoffset
			self.yoffset = best_yoffset
			self.radius = best_radius
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(best_x), int(best_y)), int(best_radius), # Draws the yellow circle on video stream
					(0, 255, 255), 2)
			cv2.circle(frame, best_center, 5, (0, 0, 255), -1) # Draws a red dot in the center of the yellow circle
		else:
			self.xoffset = 0
			self.yoffset = 0
			self.radius = 0

		return self.xoffset, self.yoffset, self.radius #feed the optimized xoffset and yoffset to telloCV.py
	