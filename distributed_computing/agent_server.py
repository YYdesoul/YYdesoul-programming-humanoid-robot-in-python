'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import xmlrpclib
from recognize_posture import PostureRecognitionAgent
import threading

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2', )

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.target_joints.get(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        
    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.angle_interpolation(keyframes, self.perception)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, transform)
        
if __name__ == '__main__':
    agent = ServerAgent()
    server = SimpleXMLRPCServer(("localhost", 6666), requestHandler=RequestHandler)
    server.register_function(ServerAgent.get_angle, 'get_angle')
    server.register_function(ServerAgent.set_angle, 'set_angle')
    server.register_function(ServerAgent.get_posture, 'get_posture')
    server.register_function(ServerAgent.execute_keyframes, 'execute_keyframes')
    server.register_function(ServerAgent.get_transform, 'get_transform')
    server.register_function(ServerAgent.set_transform, 'set_transform')       
    server.register_introspection_functions()
    server.register_multicall_functions()
    server.register_instance(agent)
    t = threading.Thread(target=server.serve_forever)
    t.start()
    print "server is ready on localhost:6666"
    agent.run()

