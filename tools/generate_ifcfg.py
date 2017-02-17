# generates ifcfg files for USRP x300s

import string

PORTS_PER_CARD = 2
SUBNET_BASE = 40

interface_bases = ['enp5s0f']

macaddrs = ['68:05:ca:32:39:38',\
            '68:05:ca:32:39:39']

ifcfg_template = '''
TYPE="Ethernet"
BOOTPROTO="none"
IPADDR0="192.168.$SUBNET.1"
DEFROUTE="yes"
IPV4_FAILURE_FATAL="no"
IPV6INIT="no"
IPV6_FAILURE_FATAL="no"
NAME="$NETWORKINTERFACE"
BOOTPROTO="none"
ONBOOT="yes"
HWADDR"$MACADDR"
PEERDNS="yes"
PEERROUTES="yes"
ZONE="trusted"
MTU="3500"
NM_MANAGED="no"
'''

interfaces = [ibase + str(p) for ibase in interface_bases for p in range(PORTS_PER_CARD)]

for i, interface in enumerate(interfaces):
        subnet = i + SUBNET_BASE
        macaddr = macaddrs[i]
        ifcfg = string.Template(ifcfg_template)
        ifcfg = ifcfg.substitute(\
                        SUBNET = subnet,\
                        MACADDR = macaddr,\
                        NETWORKINTERFACE = interface)
        f = open('ifcfg-' + interface, 'w')
        f.write(ifcfg)
        f.close()
        
         
        
