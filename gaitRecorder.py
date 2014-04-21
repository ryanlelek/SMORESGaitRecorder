#!/usr/bin/env python

"""
This is a gait recorder of the SMORES robot in Gazebo
For more information, please visit https://github.com/jimjing/SMORESGaitRecorder
This tool uses Python versions of all defined Gazebo protobuf messages
The GUI of this tool is supported by Panda3d (panda3d.org)

Last update: Apr. 20th, 2014
Author: Jim Jing
"""

# For panda3d
import direct.directbase.DirectStart as ds
from direct.gui.DirectGui import *
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import *
try:
    ds.go()
except Exception:
    pass

# General Importing
import socket
import time
import sys
import re
import threading
import copy
from datetime import datetime

# Python version of gazebo protobuf
from proto.packet_pb2     import Packet
from proto.publish_pb2    import Publish
from proto.request_pb2    import Request
from proto.response_pb2   import Response
from proto.pose_pb2       import Pose
from proto.vector2d_pb2   import Vector2d
from proto.subscribe_pb2  import Subscribe


class Module(object):
    def __init__(self, name, ID):
        """
        A SMORES Module
        """
        self.name = name
        self.ID = ID
        self.joints = [0.0,0.0,0.0,0.0]; # x,y,z,w

class GzCommunicator(object):
    def __init__(self):
        """Init function"""
        self.MASTER_TCP_IP   = '127.0.0.1'
        self.MASTER_TCP_PORT = 11345

        self.NODE_TCP_IP     = '127.0.0.1'
        self.NODE_TCP_PORT   = 11451

        self.TCP_BUFFER_SIZE = 40960

    def StartCommunicator(self, module_name):
        """Start the communication through gztopic"""
        # Listen for Subscribers
        s_sub = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s_sub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s_sub.bind((self.NODE_TCP_IP, self.NODE_TCP_PORT))
        s_sub.listen(5)


        # Register as a Publisher with Gazebo
        pk            = Packet()
        pk.stamp.sec  = int(time.time())
        pk.stamp.nsec = datetime.now().microsecond
        pk.type       = "advertise"

        pub           = Publish()
        pub.topic     = "/gazebo/{}/gaitRecorder".format(module_name, module_name)
        pub.msg_type  = "4"
        pub.host      = self.NODE_TCP_IP
        pub.port      = self.NODE_TCP_PORT

        pk.serialized_data = pub.SerializeToString()

        self.s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_reg.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # try to connect to gazebo server
        succ = False
        while not succ:
            try:
                self.s_reg.connect((self.MASTER_TCP_IP, self.MASTER_TCP_PORT))
            except Exception as e:
                print "Waiting for Gazebo server..."
                time.sleep(2)
            else:
                succ = True

        self.s_reg.send(hex(pk.ByteSize()).rjust(8))
        self.s_reg.send(pk.SerializeToString())

        print "Wating for subscriber... "
        # Respond to a subscriber
        try:
            self.conn, address = s_sub.accept()
            data = self.conn.recv(self.TCP_BUFFER_SIZE)
        except Exception as e:
            print "Lost connection to Gazebo server."
            print e
            print

        else:
            # Decode Incomming Packet
            pk_sub = Packet()
            pk_sub.ParseFromString(data[8:])
            print "Packet:\n", pk_sub

            # Decode Subscription Request
            sub = Subscribe()
            sub.ParseFromString(pk_sub.serialized_data)
            print "Sub:\n", sub
            print

    def stop(self):
        """Stop the communicator"""
        self.s_reg.close()

    def SendJointValues(self,name, x, y, z, w):
        """Function used to send joints command to gztopic"""
        # Pack Data for Reply
        cmd_vel = Pose()
        cmd_vel.name = name
        cmd_vel.position.x = 1.0
        cmd_vel.position.y = 1.0
        cmd_vel.position.z = 1.0

        cmd_vel.orientation.x = x
        cmd_vel.orientation.y = y
        cmd_vel.orientation.z = z
        cmd_vel.orientation.w = w

        # Publish Packet to Subscriber
        pk_pub            = Packet()
        pk_pub.stamp.sec  = int(time.time())
        pk_pub.stamp.nsec = datetime.now().microsecond
        pk_pub.type       = Vector2d.DESCRIPTOR.full_name
        pk_pub.serialized_data = cmd_vel.SerializeToString()

        self.conn.send(hex(cmd_vel.ByteSize()).rjust(8))
        self.conn.send(cmd_vel.SerializeToString())

        time.sleep(0.2)

class GUI(object):
    def __init__(self):
        #add some text
        bk_text = "Control Editor"
        self.textObject = OnscreenText(text = bk_text, pos = (0.95,-0.95),scale = 0.07,fg=(1,0.5,0.5,1),align=TextNode.ACenter,mayChange=1)
        self.slider1 = DirectSlider(pos = (0,0,0),range=(-1.5,1.5), value=0, pageSize=3, command=self.showValue)
        self.slider2 = DirectSlider(pos = (0,0,0.2),range=(-1.5,1.5), value=0, pageSize=3, command=self.showValue)
        self.slider3 = DirectSlider(pos = (0,0,0.4),range=(-1.5,1.5), value=0, pageSize=3, command=self.showValue)
        self.slider4 = DirectSlider(pos = (0,0,0.6),range=(-1.5,1.5), value=0, pageSize=3, command=self.showValue)

        self.textObject_s1 = OnscreenText(text = str(0), pos = (-1.1,0.0,0),scale = 0.07,fg=(1,0.5,0.5,1),align=TextNode.ACenter,mayChange=1)
        self.textObject_s2 = OnscreenText(text = str(0), pos = (-1.1,0.2,0),scale = 0.07,fg=(1,0.5,0.5,1),align=TextNode.ACenter,mayChange=1)
        self.textObject_s3 = OnscreenText(text = str(0), pos = (-1.1,0.4,0),scale = 0.07,fg=(1,0.5,0.5,1),align=TextNode.ACenter,mayChange=1)
        self.textObject_s4 = OnscreenText(text = str(0), pos = (-1.1,0.6,0),scale = 0.07,fg=(1,0.5,0.5,1),align=TextNode.ACenter,mayChange=1)
#         #add button
#         self.b = DirectEntry(pos = (0.0,0.0,-0.4), text = "" ,scale=.05,command=self.setText,
#         initialText="Type Something", numLines = 2,focus=1,focusInCommand=self.clearText)

        self.save_pose_button = DirectButton(pos = (0.0,0.0,-0.6), text = ("Save Pose", "Saved", "Save Pose", "disabled"), scale=.05, command=self.savePose)
        self.save_cmd_button = DirectButton(pos = (-0.6,0.0,-0.6), text = ("Save Command", "Saved", "Save Command", "disabled"), scale=.05, command=self.saveCommandToFile)


        self.modules = []
        self.saved_pose = []
        self.loadConfig()

        # Create a frame
        self.menu = DirectOptionMenu(pos = (0.0,0.0,-0.4),text="options", scale=0.1,items=[m.name for m in self.modules],initialitem=2,
        highlightColor=(0.65,0.65,0.65,1),command=self.itemSel)

        self.current_m = self.modules[0]
        self.menu.set(0)

        self.communicator = GzCommunicator()
        self.communicator.StartCommunicator(self.modules[0].name)

#         self.send_joint_thread = threading.Thread(target = self.SendJointValues)
#         self.send_joint_thread.daemon = True
#         self.send_joint_thread.start()

    def SendJointValues(self):
        name = str(self.current_m.ID)
        x = self.current_m.joints[0]
        y = self.current_m.joints[1]
        z = self.current_m.joints[2]
        w = self.current_m.joints[3]

        self.communicator.SendJointValues(name, x,y,z,w)

    def updateJointValues(self):
        self.slider1['value'] = self.current_m.joints[0]
        self.slider2['value'] = self.current_m.joints[1]
        self.slider3['value'] = self.current_m.joints[2]
        self.slider4['value'] = self.current_m.joints[3]

    def loadConfig(self):
        """
        Load the InitialConfiguration file
        """
        try:
            f = open("InitialConfiguration","r")
        except IOError:
            print "Please copy the InitialConfiguration to the Gait Recorder folder."
            sys.exit(1)

        ID = 0
        data = f.readlines()
        for line in data:
            r = re.match(r"\t*<name>(?P<m_name>\w+)<\/name>",line)
            if r:
                name = r.group("m_name")
                if name not in [m.name for m in self.modules]:
                    self.modules.append(Module(name, ID))
                    ID += 1

        f.close()

    def savePose(self):
        for m in self.modules:
            name = str(m.ID)
            x = m.joints[0]
            y = m.joints[1]
            z = m.joints[2]
            w = m.joints[3]
            data = "{} {:.3} {:.3} {:.3} {:.3}\n".format(name,x,y,z,w)
            self.saved_pose.append(data)

    def saveCommandToFile(self):
        f = open("Commands","w+")
        for data in self.saved_pose:
            f.write(data)
        f.close()

    def getModuleWithName(self, name):
        return [m for m in self.modules if m.name == name][0]


    #callback function to set  text 
    def setText(self,textEntered):
        self.textObject.setText(textEntered)
        self.name = textEntered

    #clear the text
    def clearText(self):
        self.b.enterText('')

    # Callback function to set  text 
    def itemSel(self, arg):
        self.current_m = self.getModuleWithName(arg)
        self.updateJointValues()

    def showValue(self):
        name = str(self.current_m.ID)
        x = float(self.slider1['value'])
        y = float(self.slider2['value'])
        z = float(self.slider3['value'])
        w = float(self.slider4['value'])
        self.textObject_s1.setText("Body {:.3}".format(x))
        self.textObject_s2.setText("Left {:.3}".format(y))
        self.textObject_s3.setText("Right {:.3}".format(z))
        self.textObject_s4.setText("Front {:.3}".format(w))

        if self.current_m.joints != [x,y,z,w]:
            self.current_m.joints = [x,y,z,w]
            self.communicator.SendJointValues(name, x,y,z,w)

    def run(self):
        run()

if __name__=="__main__":
    try:
        g = None
        g = GUI()
        g.run()
    except (KeyboardInterrupt, SystemExit):
        raise
    finally:
        if g:
            g.communicator.stop()
