#!/usr/bin/env python
#encode: UTF-8
import rospy
from std_msgs.msg import String
import subprocess

def jtalk(message):
    t=message.data.encode('utf-8')
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t)
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)

if __name__ == '__main__':
    rospy.init_node('talker')
    rospy.Subscriber('message',String,jtalk)
    rospy.spin()
