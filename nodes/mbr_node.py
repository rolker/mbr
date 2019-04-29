#!/usr/bin/env python

import mbr
import rospy
from std_msgs.msg import Float32

class RadioLogger:
    def __init__(self, sn):
        self.sn = sn
        self.radio = mbr.radio_api_lib.radio(sn)
        self.topic = '/mbr/'+str(sn)

    def run(self):
        pubs = {}
        
        while not rospy.is_shutdown():
            ws = self.radio.get_wireless_status()
            print ws
            for remote in ws['wireless_status'].keys():
                if not remote in pubs:
                    pubs[remote] = {}
                remote_str = str(int(remote))
                rstatus = ws['wireless_status'][remote]
                for k in rstatus.keys():
                    if not k in pubs[remote]:
                        pubs[remote][k] = rospy.Publisher(self.topic+'/'+remote_str+'/'+k, Float32, queue_size=10)
                    pubs[remote][k].publish(rstatus[k])
            rospy.sleep(0.1)
            
        
while not rospy.is_shutdown():
    rospy.init_node('mbr')
    sns = mbr.radio_api_lib.discover()
    print sns
    if len(sns) > 0:
        logger = RadioLogger(sns[0])
        logger.run()
        
