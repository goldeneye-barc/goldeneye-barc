import rosbag
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from itertools import cycle
from collections import defaultdict, OrderedDict
import pickle

file_path = sys.argv[1]
file_name = sys.argv[2]
#bag = rosbag.Bag('../camera_lidar_gps_exp2/2016-02-18-14-14-29.bag') 
bag = rosbag.Bag(file_path + "/" + file_name) 

# GET METADATA
_topics = ['/scan']
metadata = {}
# hedge pos metadata
metadata['/hedge_pos_a'] = {'vehicle_hedge': 2, 'obstacle_hedge': 4}
# scan metadata
_, msg, _ = next(bag.read_messages(topics=_topics))
attrs = ['angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max']
metadata['/scan'] = { attr: getattr(msg, attr) for attr in attrs }

# GET DATA
_topics = ['/hedge_pos_a', '/image_raw', '/scan']
topics = cycle(zip(_topics, range(len(_topics))))
messages = bag.read_messages(topics=_topics)
parsed = defaultdict(list)
curr_t, init_t = None, None
bridge = CvBridge()

def process(msg, topic):
	if topic == '/image_raw':
		return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	elif topic == '/scan':
		return msg.ranges #get metadata previously
	return msg
curr_t = 0
for topic, msg, t in messages:
	next_topic, i = next(topics)
	while next_topic != topic:
		try:
			topic, msg, t = next(messages)
		except StopIteration:
			break
	#print(next_topic, topic, curr_t)
	processed = process(msg, topic)
	#if topic == '/image_raw':
		#cv2.imshow('img', processed)
		#cv2.waitKey(10)
	if topic == '/hedge_pos_a':
		if not init_t:
			init_t = processed.timestamp_ms
		curr_t = processed.timestamp_ms - init_t
		#print(processed)
	parsed[curr_t].append(processed)

for k, v in parsed.items():
	if len(v) != len(_topics):
		del parsed[k]

data = OrderedDict(sorted(parsed.items(), key=lambda t: t[0]))
#data format: -> (timestamp, hedge_pos, image, lidar_ranges)
data = data.items()
print("Processed data, saving")
data = np.array(data)
output_data = open(file_path + "/output_data", 'wb')
np.save(output_data, data)
output_data.close()

