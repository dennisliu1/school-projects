import socket
import zigbee
import select
import bind_table

###############################################################################
# We need to map several pieces of data from the UDP socket.
# These dictionaries will provide a inexpensive look up table for that. 
# Declarations
###############################################################################

udp_port_dict  = {} ##Udp socket to port it was bound to
udp_lka_dict   = {} ##Udp socket to its LKA it received data from
udp_queue_dict = {} ##Udp socket to its queued up data
udp_socks      = [] ##List of all Udp sockets

zig_port_dict  = bind_table.node_list ## list of XBee address to port numbers
zig_data_queue = []                   ## Data/address queued for the XBee socket

MAX_UDP_SIZE = 8192 ## Maximum UDP packet size to read/write
MAX_ZIG_SIZE = 84   ## Maximum XBee packet size to read/write

end_point  = 0xe8   ## Endpoint to bind the XBee socket to
profile_id = 0xc105 ## Profile ID to bind the XBee socket to
cluster_id = 0x11   ## Cluster ID to bind the XBee socket to