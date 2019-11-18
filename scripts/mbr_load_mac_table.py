#!/usr/bin/env python

import mbr
import sys
import json

if len(sys.argv) == 3:
    radio_sn = int(sys.argv[1])
    infile = sys.argv[2]
    print 'radio:' , radio_sn, '\tinfile:', infile
    mac_routes = json.load(open(infile))
    print mac_routes

    radio = mbr.radio_api_lib.radio(radio_sn)
    for ip,mac in mac_routes['mac_routes'].iteritems():
        print ip,mac
        radio.add_host(ip, mac)
else:
    print 'usage: mbr_load_mac_table radio_sn infile'
    
