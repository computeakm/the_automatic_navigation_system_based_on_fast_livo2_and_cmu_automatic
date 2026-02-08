import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ht/ht_ws/install/realsense2_ros_mqtt_bridge'
