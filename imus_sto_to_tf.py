#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import sys, traceback
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import Header

class Imu:
    def __init__(self, name):
        self.name = name
        self.q_indexes = [-1,-1,-1,-1]
        self.t = 0
        self.q = [-1,0,0,0]

    def __repr__(self):
        return self.__str__()
    def __str__(self):
        return f"{self.name}, ({self.q_indexes})\n{self.t}:{self.q}"

##I need this to be able to see anything..

# I had fixed this before, but whatever.
translation_dict = {
        "torso":(0,0,1.4),
        "pelvis":(0,0,1),
        "femur_r":(0.0,-0.15,0.7),
        "femur_l":(0.0, 0.15,0.7),
        "tibia_r":(0.0,-0.15,0.3),
        "tibia_l":(0.0, 0.15,0.3),
        "talus_r":(0.1,-0.15,0.05),
        "talus_l":(0.1, 0.15,0.05),
        }



class Reader:
    def __init__(self, FILENAME, period = 0.01, repeat = True, artificial_time = True):
        """
        Reader(FILENAME, period= 0.01)
        Publishes straight tfs from imu.sto type file.

        I currently have to invert the q.w, so I wonder how am I saving this. I think I am probably using the conventional way of saving 
        """
        self.rate = rospy.Rate(1/ period) # in seconds
        self.FILENAME= FILENAME
        self.repeat = repeat
        self.labels = None
        self.artificial_time = artificial_time
        self.t0 = rospy.Time().now().to_sec()
        self.t = self.t0
        self.period = period
        self.imu_list = []
        self.imus = []
        self.tf_prefix = "imu/"
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def set_imu_names(self):
        for label in self.labels:
            label_components = label.split("_")
            label_prefix = label_components[0]
            label_restffix= label_components[1:]
            if "q1" in label_restffix:
                ##idk how to do it better
                self.imu_list.append(Imu(label.split("_q1")[0]))
        print(self.imu_list)

    def gen_capture_lists(self):
        for imu in self.imu_list:
            for i, label in enumerate(self.labels):
                if imu.name in label and "_q" in label:
                    if "_q1" in label:
                        imu.q_indexes[0] = i
                    if "_q2" in label:
                        imu.q_indexes[1] = i
                    if "_q3" in label:
                        imu.q_indexes[2] = i
                    if "_q4" in label:
                        imu.q_indexes[3] = i

    def get_qs(self,line):
        ## updata imu_list
        for imu in self.imu_list:
            #print(f"{imu.name}: {[  line[i] for i in imu.q_indexes]}")
            imu.q = [  float(line[i]) for i in imu.q_indexes]
            imu.t = line[0]
        return self.imu_list

    def gen(self):
        with open(self.FILENAME) as stofile:
            #line = csv.reader(stofile,delimiter="\t")
            stofile.readline()
            stofile.readline()
            stofile.readline()
            stofile.readline()
            #next(line) ## trying to skip the header
            #next(line) ## trying to skip the header
            #next(line) ## trying to skip the header
            #next(line) ## trying to skip the header
            
            self.labels = stofile.readline().split("\t") ## this line has the actual labels, if you want them
            self.set_imu_names()
            self.gen_capture_lists()
            for imu in self.imu_list:
                print(imu)

            ##exit()
            while not rospy.is_shutdown():
                stofile.seek(0)
                stofile.readline()
                stofile.readline()
                stofile.readline()
                stofile.readline()
                stofile.readline()
                line = stofile.readline().split("\t")
                while (len(line) > 1): 
                    #print(len(line))
                    #print("line:%s"%line)
                    if self.repeat:
                        ## need to use actual time, or it will break when i loop
                        if self.artificial_time:
                            self.t += self.period 
                        else:
                            self.t = rospy.Time().now().to_sec()
                        line[0]=str(self.t)

                    yield ["{:+.5f}".format(float(i)) for i in line]
                    line = stofile.readline().split("\t")
                if not self.repeat:
                    break
                else:
                    print("rewinding")

    #bytesToSend         = str.encode(msgFromClient)
    def loopsend(self): ## remove rate and make this guy output the values if you want to reuse this class
        #try:
            for i,msg in enumerate(self.gen()):
                ## sends tfs
                #print(msg)
                imu_curr = self.get_qs(msg)
                ## we are going to use the same header
                h = Header()
                h.stamp = rospy.Time.from_seconds(self.t)
                h.frame_id = "subject_heading"
                transforms = []
                for imu in imu_curr:
                    #print(imu)
                    this_tfs = tf2_ros.TransformStamped()
                    this_tfs.header = h
                    ## I never know the order...
                    ## but this, right now, should be the inverse transform, so the one with inverted w i believe
                    this_tfs.transform.rotation.w = -imu.q[0] 
                    this_tfs.transform.rotation.x = imu.q[1] 
                    this_tfs.transform.rotation.y = imu.q[2] 
                    this_tfs.transform.rotation.z = imu.q[3] 
                    this_tfs.transform.translation.x = translation_dict[imu.name][0]
                    this_tfs.transform.translation.y = translation_dict[imu.name][1]
                    this_tfs.transform.translation.z = translation_dict[imu.name][2]
                    this_tfs.child_frame_id = self.tf_prefix + imu.name
                    transforms.append(this_tfs)
                
                self.broadcaster.sendTransform(transforms)

                if rospy.is_shutdown():
                    break
                
                self.rate.sleep()

            rospy.loginfo("finished!")
        #except:
        #    traceback.print_exc(file=sys.stdout)



if __name__ == "__main__":
    try:
        rospy.init_node("sto_dumper")
        #A = Reader("test.sto")
        #A = Reader("/catkin_ws/Data/02_ruoli/ViconData/Ruoli/Moticon_insole/RealTimekIDS2/2023-03-03-11-53-52walking011_imus_lower.sto")
        file = "/catkin_ws/Data/02_ruoli/ViconData/Ruoli/Moticon_insole/RealTimekIDS2/2023-03-03-11-56-24walking012_imus_lower.sto"
        file = rospy.get_param("~sto_file",default= file)
        period = rospy.get_param("~period", default=0.01)
        repeat = rospy.get_param("~repeat", default=True)
        tf_prefix = rospy.get_param("~tf_prefix", default="")
        A = Reader(file,period= period, repeat=repeat)
        A.tf_prefix = tf_prefix
        A.loopsend()
    except rospy.ROSInterruptException:
        pass
