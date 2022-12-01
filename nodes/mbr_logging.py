#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import socket
import paramiko
import time

MBR_IP_ADDRESS="10.19.9.206"

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
    TX_bw_kbs,
    RX_bw_kbs,
    Sites_count
'''

# Start the MBR data broadcast
client = paramiko.client.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

rospy.loginfo("Starting MBR data broadcast...")
print("Starting MBR data broadcast...")
DATA_STARTED = False
while not DATA_STARTED:
    try:
        client.connect(MBR_IP_ADDRESS, 
        username="root", 
        password="1stx_Admin")
        transport = client.get_transport()
        channel = transport.open_session()
    except Exception as e:
        rospy.loginfo("Failed to login to MBR to start data streaming.")
        print("Failed to login to MBR to start data streaming.")
        print(str(e))
        time.sleep(20)
        break
    
    try:
        _stdin, _stdout, _stderr = client.exec_command("ps | grep analog")
        response = _stdout.read().decode()
    except Exception as e:
        rospy.loginfo("Failed to execute MBR process query.")
        print("Failed to execute MBR process query.")
        print(str(e))
        time.sleep(20)
        break

    #print(response)
    if response.find("analog -s 32768 -u 1111") != -1:
        DATA_STARTED = True
    else:
        try:
            channel.exec_command("analog -s 32768 -u 11111 >/dev/null 2>&1 &")
        except:
            rospy.loginfo("Failed to start data distribution")
            print("Failed to start data distribution")
            rospy.loginfo(_stderr.read().decode())
            time.sleep(20)

    client.close()

rospy.loginfo("MBR data broadcast started.")
print("MBR data broadcast started.")
fields = []
for f in fields_string.split(','):
    fields.append(f.strip())

insock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
insock.bind(('',11111))

rospy.init_node('mbr_logging')
pubs = {}
diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)

while not rospy.is_shutdown():
    data = insock.recv(1024)
    values = data.decode('utf8').split(',')
    sn = values[0]
    if len(values) >= 24:
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        ds = DiagnosticStatus()
        ds.name = 'mbr'
        ds.hardware_id = values[0]
        for i in range(len(fields)):
            ds.values.append(KeyValue(fields[i],values[i]))

        for i in (21,22):
            if not sn+'/'+fields[i] in pubs:
                pubs[sn+'/'+fields[i]] = rospy.Publisher('mbr/'+sn+'/'+fields[i],Float32, queue_size=10)
            pubs[sn+'/'+fields[i]].publish(float(values[i]))
        site_count = int(values[23])

        for i in range(site_count):
            base_index = 24+i*21
            if len(values) >= base_index+21:
                topic = 'mbr/'+sn+'/'+values[base_index+0]+'/mean_margin'
                if not topic in pubs:
                    pubs[topic] = rospy.Publisher(topic, Float32, queue_size=10)
                pubs[topic].publish(float(values[base_index+6]))

            ds.values.append(KeyValue(values[base_index+0]+'_mean_margin',values[base_index+6]))
        diag_array.status.append(ds)
        diagnostic_pub.publish(diag_array)
