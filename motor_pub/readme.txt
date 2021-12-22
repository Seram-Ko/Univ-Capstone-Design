1. Warping
	(1) Select four points from original image.
	(2) A ROI Square image made from the points is transformed into Rectangle image by
		getPerspectiveTransform function.


2. Distinguish Lanes
	(1) Set 2 ROIs left half of the warping image and right half of the warping image.
	(2) Use Gaussian blur and white color detection to the ROIs.
	(3) Count the number of the pixels.
	(4) If left ROI image has more than 1000 white pixels, then flag_LW = 1.
	(5) If right ROI image has more than 1000 white pixels, then flag_RW = 1.
	(6) If flag_LW and flag_RW are both 1, then we assume that car is at 2nd lane.
	(7) For other cases, we assume that car is at 1st lane.


3. Image Processing
	(1) If car is at 2nd lane
		1) Canny edge detection for two white color detected ROI images.
		2) Hough transform
			- red pixel for left white line, pink pixel for right white line.
	(2) If car is at 1st lane
		1) White color detection for warping image.
		2) Yellow color detection for warping image.
		3) Canny edge detection for two color detected image.
		4) Hough transform
			- red pixel for yellow line, pink pixel for white line.


4. Weight Method
	(1) Find red pixel for each row starting from right side of the image.
		If find one, go to the next row.
	(2) Find pink pixel for each row starting from left side of the image.
		If find one, go to the next row.
	(3) Add all the x location values of found pixels.
	(4) Divide the sum for average of the lowest row's y location that red and pink pixel found.


5. PID Control
	(1) Reference is servo motor's angle when car go straight.
	(2) Error is angle for weight and center of the image and center of the lowest row of the image.
	(3) PID control the servo.


6. Obstacle Reaction
	(1) Get a distance between car and obstacle from ultrasonic sensor.
	(2) If the distance is more than 100, nothing happens. (Operate with above codes.)
	(3) If the distance is less than 100 and more than 70, the car starts to decelerate.
		P control
	(4) If the distance is less than 70, the car evade.
		- According to "2. Distinguish Lanes",
		- If the car is at 2nd lane, the car evade to 1st lane. (Left turn)
		- If the car is at 1st lane, the car evade to 2nd lane. (Right turn)
		- After evasion, it stops for 2 seconds.


