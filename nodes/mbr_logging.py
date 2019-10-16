#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import socket

fields_string = '''
    Unit_serial_number,
    Unix_time_in_seconds,
    Milliseconds,
    lat,
    lon,
    Temperature_C,
    Input_current_mA,
    Input_voltage_mV,
    Frequency_MHz_part,
    Frequency_kHz_part,
    Product_variant_number,
    Antenna_element_gain_dB,
    Number_RX_frames_received_OK,
    Number_RX_frame_errors,
    Number_RX_CRC32_errors,
    Number_Frames_relayed_through_this_station,
    Number_TX_frames_OK,
    Number_TX_frames_dropped_due_to_buffer_overrun,
    Number_TX_frames_dropped_due_to_no_link,
    Number_TX_frames_dropped_due_to_MAC_busy,
    Number_TX_frames_with_missed_ACK,
    Total_TX_bandwidth_to_mac_kb_per_s,
    Total_RX_bandwidth_from_mac_kb_per_s,
    Sites_count
'''

fields = []
for f in fields_string.split(','):
    fields.append(f.strip())

insock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
insock.bind(('',11111))

rospy.init_node('mbr_logging')
pubs = {}

while not rospy.is_shutdown():
    data = insock.recv(1024)
    values = data.split(',')
    sn = values[0]
    for i in range(len(fields)):
        if i in (21,22):
            if not sn+'/'+fields[i] in pubs:
                pubs[sn+'/'+fields[i]] = rospy.Publisher('/mbr/'+sn+'/'+fields[i],Float32, queue_size=10)
            pubs[sn+'/'+fields[i]].publish(float(values[i]))
