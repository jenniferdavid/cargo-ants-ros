# UNDER CONSTRUCTION

geometry_msgs/Point position	# Target (x,y) position
geometry_msgs/Point velocity   	# Target (Vx,Vy) velocity
float32[36] covariances			# Target covariance in position, velocity and cross covariance.
int32 id						# Target identifier
float32 probability				# Target probability [0-1]. When close to 1, the target is more probale that is its tracked object.
