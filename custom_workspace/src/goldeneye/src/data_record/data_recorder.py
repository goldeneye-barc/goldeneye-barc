#!/usr/bin/env python
import rospy
import sys
import subprocess
import os
import datetime

DATA_TOPICS = ['camera', 'gps', 'actuation', 'lidar', 'imu', 'encoders', 'processed_image', 'velocity']

class DataCollector():
    def __init__(self, params):
        self.topic_mappings = {
            'camera': '/image_raw',
            'gps': 'TODO',
            'actuation': '/ecu_pwm',
            'lidar': 'TODO',
            'imu': 'TODO',
            'encoders': 'TODO',
            'processed_image': 'TODO',
            'velocity': '/velocity_est'
        }
        self.root_path = params['root_path']
        self.topic_booleans = {topic: params[topic] for topic in DATA_TOPICS}
        self.special_topics = ['camera']
        self.init_log_file()

    def get_timestamp(self):
        return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def init_log_file(self):
        self.start_time = self.get_timestamp()
        self.log_dir = os.path.join(self.root_path, self.start_time)
        os.makedirs(self.log_dir)
        log_file_path = os.path.join(self.log_dir, 'log')
        log_file = open(log_file_path, 'w+')
        log_file.write(self.start_time)
        log_file.write(self.topic_booleans.__str__())
        log_file.close()

    def record(self):
        rosbag_command = 'rosbag record '
        for topic, rec in self.topic_booleans.items():
            if rec and topic not in self.special_topics:
                rosbag_command += ' ' + self.topic_mappings[topic]
        print('RUNNING: ', rosbag_command)
        self.rosbag_proc = subprocess.Popen(rosbag_command, stdin=subprocess.PIPE, shell=True, cwd=self.log_dir)

        if self.topic_booleans['camera']:
            camera_command = 'rosrun image_view video_recorder image:=' + self.topic_mappings['camera'] + ' _fps:=30 _image_transport:=compressed'
            print('RUNNING: ', camera_command)
            self.video_proc = subprocess.Popen(camera_command, stdin=subprocess.PIPE, shell=True, cwd=self.log_dir)
        if self.topic_booleans['processed_image']:
            camera_command = 'rosrun image_view video_recorder image:=' + self.topic_mappings['processed_image'] + ' _fps:=30 _image_transport:=compressed'
            print('RUNNING: ', camera_command)
            self.video_proc = subprocess.Popen(camera_command, stdin=subprocess.PIPE, shell=True, cwd=self.log_dir)

    def end_all(self):
        self.rosbag_proc.kill()
        if self.topic_booleans['camera']: self.camera_command.kill()

def data_recorder():
    param_names = DATA_TOPICS + ['root_path']
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    data_collector = DataCollector(params)
    data_collector.record()
    rospy.on_shutdown(data_collector.end_all)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('data_collector')
    try: 
        data_recorder()
    except rospy.ROSInterruptException:
        pass
