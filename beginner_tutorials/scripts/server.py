#!/usr/bin/env python
from beginner_tutorials.srv import *
import rospy

class Server:
        
    def handle_return_timestamp(self, req):
        return ReturnTimeResponse(rospy.get_time());

    def return_timestamp(self):
        rospy.init_node('return_timestamp_server')
        s = rospy.Service('return_timestamp', ReturnTime, self.handle_return_timestamp)
        print "Ready to receive request for ReturnTimestamp."
        rospy.spin()

if __name__ == "__main__":
    server = Server();
    server.return_timestamp();    
