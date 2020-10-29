#!/usr/bin/env python

import mbr
import rospy
import socket
from std_msgs.msg import Float32

class RadioLogger:
    def __init__(self, sn):
        self.sn = sn
        self.radio = mbr.radio_api_lib.radio(sn)
        self.topic = '/mbr/'+str(sn)

    def run(self):
        pubs = {}
        
        while not rospy.is_shutdown():
            try:
                ws = self.radio.get_wireless_status()
                #print ws
                for remote in ws['wireless_status'].keys():
                    if not remote in pubs:
                        pubs[remote] = {}
                    remote_str = str(int(remote))
                    rstatus = ws['wireless_status'][remote]
                    for k in rstatus.keys():
                        if not k in pubs[remote]:
                            pubs[remote][k] = rospy.Publisher(self.topic+'/'+remote_str+'/'+k, Float32, queue_size=10)
                        pubs[remote][k].publish(rstatus[k])
            except socket.timeout:
                pass
            rospy.sleep(0.2)
            
        
while not rospy.is_shutdown():
    rospy.init_node('mbr')
    sns = mbr.radio_api_lib.discover()
    #print sns
    if len(sns) > 0:
        logger = RadioLogger(sns[0])
        logger.run()
        
'''
Example http post to start logging via UDP

POST /logging.cgi HTTP/1.1
Host: 10.19.9.206
User-Agent: Mozilla/5.0 (X11; Linux x86_64; rv:60.0) Gecko/20100101 Firefox/60.0
Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8
Accept-Language: en-US,en;q=0.5
Accept-Encoding: gzip, deflate
Referer: http://10.19.9.206/logging.cgi
Content-Type: multipart/form-data; boundary=---------------------------345410181533079795237063405
Content-Length: 290
Connection: keep-alive
Upgrade-Insecure-Requests: 1

-----------------------------345410181533079795237063405
Content-Disposition: form-data; name="udp_port"

11111
-----------------------------345410181533079795237063405
Content-Disposition: form-data; name="action"

start
-----------------------------345410181533079795237063405--
'''
