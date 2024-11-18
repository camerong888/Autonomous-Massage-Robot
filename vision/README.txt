Topics
        Topic	           |                 Type	                  |              Published By              |                  	Subscribed By
/camera/color/image_raw	   |          sensor_msgs/Image               |     	realsense_camera_node	       |           apriltag_node, pose_detection_node
/camera/depth/image_raw	   |          sensor_msgs/Image	              |         realsense_camera_node	       |                  pose_detection_node
/apriltag/detections	   |    apriltag_ros/AprilTagDetectionArray	  |            apriltag_node	           |           tf_broadcaster, pose_detection_node
/human_pose/camera_frame   |	      geometry_msgs/PoseArray	      |          pose_detection_node	       |                       rviz_node
/human_pose/apriltag_frame |	      geometry_msgs/PoseArray	      |          pose_detection_node	       |                       rviz_node
/tf	                       |             tf/tfMessage	              |      tf_broadcaster, apriltag_node     |                    	rviz_node