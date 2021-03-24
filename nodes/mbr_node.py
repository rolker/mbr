#!/usr/bin/env python

import mbr
import rospy
import socket
from std_msgs.msg import Float32

class RadioLogger:
    def __init__(self, sn):
        self.sn = sn
        self.radio = mbr.radio_api_lib.radio.radio(sn)
        self.topic = 'mbr/'+str(sn)
        self.pubs = {}

    def runOnce(self):
        try:
            ws = self.radio.get_wireless_station_list()
            #print ws
            for remote in ws['wireless_status'].keys():
                if not remote in self.pubs:
                    self.pubs[remote] = {}
                remote_str = str(int(remote))
                rstatus = ws['wireless_status'][remote]
                for k in rstatus.keys():
                    if not k in self.pubs[remote]:
                        self.pubs[remote][k] = rospy.Publisher(self.topic+'/'+remote_str+'/'+k, Float32, queue_size=10)
                    self.pubs[remote][k].publish(rstatus[k])
        except socket.timeout: 
            pass
        except mbr.radio_api_lib.utilities.RadioException:
            pass
            
        
while not rospy.is_shutdown():
    rospy.init_node('mbr')
    sns = None
    try:
        sns = rospy.get_param('~radioSerialNumbers')
        #print(sns)
    except KeyError:
        print("No radio specified in radioSerialNumbers parameter, discovering...")
    if sns is None:      
        sns = mbr.radio_api_lib.utilities.discover()
    print(sns)
    loggers = []
    for sn in sns:
        loggers.append(RadioLogger(sn))

    if(len(loggers)):
      while not rospy.is_shutdown():
        for l in loggers:
            l.runOnce()
        rospy.sleep(0.2)

        
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
