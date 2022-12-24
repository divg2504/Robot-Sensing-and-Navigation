<launch>

	<arg name="port" />
	<arg name="freq" default="1" />
	<arg name="rec_gps" default="True" />
	<arg name="gps_moving" default="False" />
<<<<<<< HEAD
	<node name="gps_driver" pkg="gps_driver" type="gps_driver.py" args="$(arg port) $(arg freq)" output="log"/>
=======
	<node name="gps_driver" pkg="gps_driver" type="GPS_driver.py" args="$(arg port) $(arg freq)" output="log"/>
<!--
>>>>>>> b2013f3ecfd562dd706743ce8c7c8ae12357738d
	<group if="$(arg gps_moving)">
		<node name="bag_record" pkg="rosbag" type="record" args=" -O /home/divi/eece5554/LAB1/src/data/moving_data gps" />
	</group>
	<group unless="$(arg gps_moving)">
		<node name="bag_record" pkg="rosbag" type="record" args=" -O /home/divi/eece5554/LAB1/src/data/stationary_data gps" />
	</group>
-->
</launch>
