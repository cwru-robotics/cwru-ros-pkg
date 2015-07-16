try:
	import netifaces
except ImportError:
	print " "
	print "ERROR! netifaces module not found! "
	print "Please install python-netifaces."
	print "This can be done by running the following command on "
	print "Ubuntu 9.10 (and beyond)"
	print " "
	print "sudo apt-get install python-netifaces"
	print " "
	exit()
for ifaceName in netifaces.interfaces():
	if netifaces.AF_INET in netifaces.ifaddresses(ifaceName):
		for link in netifaces.ifaddresses(ifaceName)[netifaces.AF_INET]:
			if '192.168' in str(link['addr']):
				print str(link['addr'])
