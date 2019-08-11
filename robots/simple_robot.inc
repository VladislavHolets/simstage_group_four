define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 240.0
		samples 240
	)

	color "green"
	size [0.15 0.1 0.05]

	block(
		points 8
		point[0] [1 1]
		point[1] [2 0]
		point[2] [3 0]
		point[3] [4 1]
		point[4] [4 2]
		point[5] [3 3]
		point[6] [2 3]
		point[7] [1 2]
		z [0 1]
	)
)
define wingr actuator(
	block(
		points 3
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		
		z [0 1]
	
	)
	color "white"	
	size [0.1 0.05 0.02]
	
	
)

define wingl actuator(
	block(
		points 3
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 0]
		
		
		z [0 1]
	
	)
	color "white"	
	size [0.1 0.05 0.02]
	
	
)

define firstlegr actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	
	)
	color "green"	
	size [0.02 0.05 0.02]
	
	
)
define secondlegr actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	
	)
	color "green"	
	size [0.02 0.05 0.02]
	
	
)	
define firstlegl actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	
	)
	color "green"	
	size [0.02 0.05 0.02]
	
	
)
define secondlegl actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	
	)
	color "green"	
	size [0.02 0.05 0.02]
	
	
)
define eyel actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
	color "red"	
	size [0.02 0.02 0.02]
)
define eyer actuator(
	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
	color "red"	
	size [0.02 0.02 0.02]
)

define simple_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [0.05 0.05 0.05]
	gui_nose 1
	drive "diff"
	color "green"
	wingr(pose [-0.0 -0.075 -0.025 0.0])
	wingl(pose [-0.0 0.075 -0.025 0.0])
	firstlegr(pose [0.05 -0.06 -0.05 45])
	secondlegr(pose [-0.05 -0.06 -0.05 135])
	firstlegl(pose [0.05 0.06 -0.05 135])
	secondlegl(pose [-0.05 0.06 -0.05 45])
	laser(pose [0.0 0.0 -0.05 0.0])
	eyel(pose [0.07 0.025 -0.021 90])
    eyer(pose [0.07 -0.025 -0.021 90])
)	
