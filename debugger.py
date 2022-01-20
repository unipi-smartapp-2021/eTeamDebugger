import sys, os, time
from datetime import datetime
from subprocess import Popen
from PySide6.QtWidgets import QApplication, QMainWindow, QInputDialog, QFileDialog
from PySide6.QtCore import QFile
from ui_main import Ui_eTeamDebugger
import rvizconfigurer
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from tf2_msgs.msg import TFMessage
import pickle

tfs = {} # last TF for each node
msg_storage = {} # last msg for every topic {topic->msg}
msg_listener = {} # listeners {topic->listener}
snaps = {} # snapshots saved {timestamp -> {topics, timestamp, msgs}}
rospy.init_node("eTeamDebugger")

class Listener():
    topic_name = None
    topic_type = None
    subscriber = None

    def setup(self,topic_name, topic_type):
        # setting listener vars and starting it
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.subscriber = self.listen()

    def callback(self,data):
        if (self.topic_name == "/tf"):
            # if topic is /tf i will store it in the custom dict
            child = data.transforms[0].child_frame_id
            tfs[child] = data
        else:
            # otherwise in the normal dict
            msg_storage[self.topic_name] = data
    
    def listen(self):
        # start and return ros subscriber
        return rospy.Subscriber(self.topic_name, self.topic_type, self.callback)
    
    def send(self, msg, newname):
        # send out a specific message
        if (self.topic_name == "/tf"):
            # if topic name is /tf i will send all the tfs of each children inside global tfs ignoring msg and newname param
            pub = rospy.Publisher(self.topic_name, self.topic_type, latch=False, queue_size=20)
            for tf in tfs.values():
                pub.publish(tf)
                print("Sending tf with child: " + tf.transforms[0].child_frame_id)
        else:
            # otherwise i will publish the single message passed
            pub = rospy.Publisher(newname, self.topic_type, latch=True, queue_size=1)
            pub.publish(msg)
            print("Sending msg to " + newname)

    def stop(self):
        # stop and remove ros subscriber
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
    
    def resume(self):
        # recreate ros subscriber
        if self.subscriber is None:
            self.subscriber = self.listen()
            print("Resuming " + self.topic_name)


class MainWindow(QMainWindow):
    currentRec = None
    topicType = {}    

    def __init__(self):
        # setting up ui
        super(MainWindow, self).__init__()
        self.ui = Ui_eTeamDebugger()
        self.ui.setupUi(self)
        # binding buttons to py functions
        self.ui.addtopic.clicked.connect(self.addtopic)
        self.ui.removetopic.clicked.connect(self.removetopic)
        self.ui.newsnap.clicked.connect(self.newsnap)
        self.ui.removesnap.clicked.connect(self.removesnap)
        self.ui.playsnap.clicked.connect(self.playsnap)
        self.ui.updatetf.clicked.connect(self.updatetf)
        self.ui.exportsnapshots.triggered.connect(self.exportsnapshots)
        self.ui.exporttopic.triggered.connect(self.exporttopic)
        self.ui.importsnapshots.triggered.connect(self.importsnapshots)
        self.ui.importtopic.triggered.connect(self.importtopic)
        self.updatesnap()
        # creating default TF listener
        l = Listener()
        l.setup("/tf", TFMessage)
        msg_listener["/tf"] = l
        self.ui.topiclist.addItem("/tf")
        msg_storage["/tf"] = None
        self.topicType["/tf"] = TFMessage

    def addtopic(self):
        # displaying dialog for topic name
        text, ok = QInputDialog.getText(self, 'Add new topic', 'Insert topic Path.\nRemember the initial /')
        if ok:
            typ = ""
            allowed_type = ["LaserScan","PointCloud2", "TF", "Image"]
            while True:
                typ, ok2 = QInputDialog.getText(self, 'Add new topic', 'Insert the type from [LaserScan, PointCloud2, TF, Image]')
                if ok:
                    # checking if value is in 
                    if (typ in allowed_type):
                        # updating ui and and global vars
                        self.ui.topiclist.addItem(text)
                        self.topicType[text] = typ
                        # creating listener
                        if typ == "LaserScan":
                            typ = LaserScan
                        elif typ == "PointCloud2":
                            typ = PointCloud2
                        elif typ == "TF":
                            typ = TFMessage
                        elif typ == "Image":
                            typ = Image
                        l = Listener()
                        l.setup(text, typ)
                        msg_listener[text] = l
                        break;
                    else:
                        break; # user canceled
                else:
                    break; # user canceled
        

    def removetopic(self):
        # getting selected topic
        sel = self.ui.topiclist.currentItem() 
        if sel is None:
            return
        # removing it from ui and from listeners
        self.ui.topiclist.takeItem(self.ui.topiclist.currentRow())
        msg_listener[sel.text()].stop()
    
    def newsnap(self):
        # create a new snapshot
        # list every topic in the list and getting current timestamp
        items = [self.ui.topiclist.item(x).text() for x in range(self.ui.topiclist.count())]
        ts = datetime.now().isoformat()
        # storing each msg needed in snap
        msgs = {}
        for itm in items:
            if itm in msg_storage:            
                msgs[itm] = msg_storage[itm]
        new = {"topics": items, "ts": ts, "msgs": msgs}
        snaps[ts] = new
        # updating ui
        self.updatesnap()

    def removesnap(self):
        # remove selected snapshot
        # getting selected snapshot
        sel = self.ui.snaplist.currentItem()
        if sel is None:
            return
        # removing it from global var and update ui
        del snaps[sel.text()]
        self.updatesnap()
    
    def playsnap(self):
        # broadcast snapshot
        # getting selected snapshot
        sel = self.ui.snaplist.currentItem() 
        if sel is None:
            return
        # retrive needed vars
        sel = sel.text()
        topics = snaps[sel]["topics"]        
        sel_safe = self.safe_topic(sel) # transforming timestamp in an allowed topic form
        
        # checking if RVIZ Open is selected
        if (self.ui.openrviz.isChecked()):
            # creating RVIZ config file for each topic in the snapshot
            rviz = rvizconfigurer.get_general()
            for topic in topics:
                new_name = topic + sel_safe
                if self.topicType[topic] == "PointCloud2":
                  rviz += rvizconfigurer.get_PointCloud(new_name)
                elif self.topicType[topic] == "LaserScan":
                  rviz += rvizconfigurer.get_LaserScan(new_name)
                elif self.topicType[topic] == "Image":
                  rviz += rvizconfigurer.get_Image(new_name)
            rviz += rvizconfigurer.get_end(self.ui.tflist.currentItem())
            
            # storing the snapshot config file
            filename = "snapshots/snapshot"+sel+".rviz"
            f = open(filename, "w")
            f.write(rviz)
            f.close()
            # opening RVIZ and waiting for its opening
            Popen("rosrun rviz rviz -d '"+filename+"'", shell=True)
            time.sleep(2)
        # publishing each msg in snapshot
        for topic in topics:
            l = msg_listener[topic]
            if topic == "/tf": 
                l.send(None, "") # for /tf i don't need msg and newname
            else:
                new_name = topic + sel_safe # creating new topic name with topic + safe form of timestamp
                if topic in snaps[sel]["msgs"]:
                    l.send(snaps[sel]["msgs"][topic], new_name)
                else:
                    print("No " + topic + " data stored in this snapshot!")

    def updatetf(self):
        # update ui with stored tfs
        self.ui.tflist.clear()
        for tf in tfs.keys():
            self.ui.tflist.addItem(tf)

    def exportsnapshots(self):
        # export snapshots and topic list to file
        self.stopListeners()
        tostore = [tfs, msg_storage, msg_listener, snaps, self.topicType]
        self.export(tostore, "Save Topics and Snapshots")

    def exporttopic(self):
        # export topic list to file
        self.stopListeners()
        tostore = [tfs, self.topicType, msg_listener]
        self.export(tostore, "Save Topics")

    def importsnapshots(self):
        # import snapshots and topic list from file
        global tfs
        global msg_listener
        global snaps
        global msg_storage
        data = self.imp("Import Topics and Snapshots")
        if data is None:
            return
        self.stopListeners()

        # copying things to vars and updating ui
        tfs = data[0]
        msg_storage = data[1]
        msg_listener = data[2]
        snaps = data[3]
        self.topicType = data[4]
        #rebuilding ui and other vars
        self.ui.topiclist.clear()
        self.ui.tflist.clear()
        self.ui.snaplist.clear()
        for k in self.topicType.keys():
            self.ui.topiclist.addItem(k)
        for k in tfs.keys():
            self.ui.tflist.addItem(k)
        for k in snaps.keys():
            self.ui.snaplist.addItem(k)
        for l in msg_listener.values():
            l.resume()

    def importtopic(self):
        # import topic list from file
        global tfs
        global msg_listener
        global snaps
        global msg_storage

        data = self.imp("Import Topics")
        if data is None:
            return
        self.stopListeners()

        # copying things to vars and updating ui
        tfs = data[0]
        self.topicType = data[1]
        msg_listener = data[2]
        print(msg_listener)
        self.ui.topiclist.clear()
        self.ui.tflist.clear()
        self.ui.snaplist.clear()
        snaps = {}
        msg_storage = {}
        for k,v in self.topicType.items():
            self.ui.topiclist.addItem(k)
        for k in tfs.keys():
            self.ui.tflist.addItem(k)
        for l in msg_listener.values():
            l.resume()
        

    def closeEvent(self, event):
        # stopping listeners on closing
        self.stopListeners()
        event.accept()

    #utils
    def imp(self, title):
        # show open file dialog and depickle selected file
        name = QFileDialog.getOpenFileName(self, title)
        if name[0] != "":
            with open(name[0], 'rb') as fp:
                return pickle.load(fp)
        return None

    def export(self, data, title):
        # show save file dialog and pickle passed data
        name = QFileDialog.getSaveFileName(self, title)
        if name[0] != "":
            with open(name[0], 'wb') as fp:
                pickle.dump(data, fp, protocol=pickle.HIGHEST_PROTOCOL)
            self.resumeListeners()

    def stopListeners(self):
        # stop all listeners
        for k,v in msg_listener.items():
            v.stop()

    def resumeListeners(self):
        # resume all listeners
        for k,v in msg_listener.items():
            v.resume()

    def updatesnap(self):
        # update ui snaps list 
        self.ui.snaplist.clear()
        for k in snaps.keys():
            self.ui.snaplist.addItem(k)

    def get_topics(self):
        # get all topics from ui topics list
        topics = []
        for i in range(self.ui.topiclist.count()):
            topics.append(self.ui.topiclist.item(i).text())
        return topics

    def safe_topic(self, name):
        # remove banned chars for topic name
        return name.replace(":","").replace("-","").replace(".","")

if __name__ == "__main__":
    # launching  GUI
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


