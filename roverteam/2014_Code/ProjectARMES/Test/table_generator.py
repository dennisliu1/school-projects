import zigbee

stored_nodes = {}
port = 4000
output_file = "bind_table.py"
retries = 5

for i in xrange(0, retries):
  
  new_nodes = zigbee.getnodelist()
  
  for node in new_nodes:
    if node.addr_extended not in stored_nodes:
      stored_nodes[node.addr_extended] = port
      port += 1

fh = open("WEB/python/%s" %output_file, 'w')
fh.write("node_list = {\r\n")

key_list = stored_nodes.keys()
for key in key_list:
  if key == key_list[-1]:
    fh.write("\t \"%s\":%d\r\n" %(key, stored_nodes[key]))
  else:
    fh.write("\t \"%s\":%d,\r\n" %(key, stored_nodes[key]))

fh.write("}\r\n")
fh.close()

print "Wrote %d entries" %len(key_list)