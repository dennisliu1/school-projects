""" This script is intended for demonstration purposes.  It meets the minimum
requirements needed to move the data from the UDP network to the XBee network.
 
A more robust application may have additional error checking, command line
argument support or another features beyond this.
 
The basics of this application are to create a number of UDP sockets that
are logically connected to a like number of XBee nodes connected to the 
Gateway product.  Traffic received on the UDP socket will be sent over the
XBee network to the paired XBee node, and likewise traffic received from the
XBee node will be sent to the paired socket.
 
The UDP sockets use the 'last known address'(LKA) to know the destination to 
send the XBee data to.  The LKA is the last received UDP packet on that socket
source address, and the script will blindly send data back to that address,
regardless of whether or not it is listening. 
 
Initially, the UDP socket does not have a LKA, and any received data that would
normally be forwarded to the LKA is dropped.  To set the LKA of a UDP socket to
stop forwarding data from the XBee network, send it a UDP packet of length 0.
 
NOTE:  There is no limit to how much the application will attempt to queue up
to send to the XBee network or the UDP network.  If too much is queued up, 
unexpected errors may occur, resulting in strange python behavior, device 
panicing, or loss of data.
"""
 
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
 
###############################################################################
# To populate the above dictionaries with relevent data
# All data is based off the address to port dictionary in bind_table
###############################################################################
 
for item,port in zig_port_dict.items(): 
  zig_port_dict[port] = item # provide the reverse lookup from port to address 
 
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
  sock.bind(("", port))  #Create and bind a UDP socket
 
  udp_port_dict[sock] = port  #provide a udp socket to port and vise versa
  udp_port_dict[port] = sock
 
  udp_lka_dict[sock] = None   #Provide a udp socket to LKA
  udp_queue_dict[sock] = []   #Create a list object to act as a data queue
 
  udp_socks.append(sock)      #Append the socket to the UDP socket list
 
 
#Create and bind the XBee socket
zig_sock = socket.socket(socket.AF_ZIGBEE, socket.SOCK_DGRAM, socket.ZBS_PROT_TRANSPORT)
zig_sock.bind(("", end_point, profile_id, cluster_id)) 
 
#Create a list of the udp_sockets plus XBee socket to monitor
sock_list = udp_socks + [zig_sock]
 
###############################################################################
# For the main portion of the script, we perform a select call on the list of
# sockets, and must handle 4 distinct results from the select call.
#  - XBee data to read
#  - XBee data to write
#  - UDP data to read
#  - UDP data to write
###############################################################################
 
print "Starting main"
while True:
  rl, wl, el = select.select(sock_list, sock_list, [])
 
  #############################################################################
  # XBee data to read
  # Read the data, if the address read from is mapped to a UDP socket,
  # Queue the data up for that socket's data queue
  ###############################################################################
 
  if zig_sock in rl:    
    data, addr = zig_sock.recvfrom(255)
    print "Received %d bytes from address: " %len(data), addr
    if addr[0] in zig_port_dict:
      port = zig_port_dict[addr[0]]
      udp_queue_dict[udp_port_dict[port]].append(data)
    else:
      print "Unknown zigbee node contacted us!"
 
  #############################################################################
  # XBee data to write
  # If we have data to write, send data to the address specified in the queue's
  # tuple.  Save any data that wasn't sent, and if empty, pop the element off
  #############################################################################
 
  if zig_sock in wl and len(zig_data_queue) != 0:    
    data, addr = zig_data_queue[0]
    segment = ((len(data) > MAX_ZIG_SIZE) and MAX_ZIG_SIZE) or len(data)      
    sent = zig_sock.sendto(data[:segment], 0, addr)
    print "Wrote %d bytes to address: " %sent, addr
    data = data[sent:]
    if len(data) == 0:
      zig_data_queue.pop(0)
    else:
      zig_data_queue[0] = (data, addr)
 
  #############################################################################
  # UDP data to read
  # Get the data and address, if the data is of length 0, remove the LKA and 
  # move on.  If the LKA address doesn't match the source address of this 
  # packet, the LKA address becomes the source address of this packet.
  # Find the UDP socket's associated XBee address, and queue it up in the
  # XBee data queue
  #############################################################################
 
  for sock in udp_socks:
    if sock in rl:      
      data, addr = sock.recvfrom(MAX_UDP_SIZE)
      print "Read %d bytes from address: " %len(data), addr 
      if len(data) == 0:
        udp_lka_dict[sock] = None
        continue
 
      if udp_lka_dict[sock] != addr:
        udp_lka_dict[sock] = addr
        udp_queue_dict[sock] = []
 
      port = udp_port_dict[sock]
      node_addr = zig_port_dict[udp_port_dict[sock]]
      zig_data_queue.append((data, (node_addr, end_point, profile_id, cluster_id)))
 
  #############################################################################
  # UDP data to write
  # If we have data to write and have a LKA address, send that LKA address 
  # the data we have queued for it.  If we have sent it all, pop off the data
  # or write back the remainder to the queue
  #############################################################################
 
  for sock in udp_socks:
    if sock in wl and len(udp_queue_dict[sock]) != 0 and udp_lka_dict[sock] is not None:      
      data = udp_queue_dict[sock][0]
      segment = ((len(data) > MAX_UDP_SIZE) and MAX_UDP_SIZE) or len(data)
 
      sent = sock.sendto(data[:segment], 0, udp_lka_dict[sock])
      print "Wrote %d bytes to address: " %sent, udp_lka_dict[sock]
      data = data[sent:]
 
      if len(data) == 0:
        udp_queue_dict[sock].pop(0)
      else:
        udp_queue_dict[sock][0] = data