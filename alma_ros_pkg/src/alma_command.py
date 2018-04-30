from subprocess import call

def main():
	print "Enter an ALMA command."
	while True:
		text = raw_input()
		call(("rostopic pub -1 /alma_node_cmd std_msgs/String -- '" + text + "'").split())

if __name__ == '__main__':
	main()
