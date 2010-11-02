#!/usr/bin/env python
import roslib
roslib.load_manifest('cwru_voice')
import rospy
import pygtk
pygtk.require('2.0')
import gtk
from std_msgs.msg import String

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

class DemoApp(object):
    """GStreamer/PocketSphinx Demo Application"""
    def __init__(self):
        """Initialize a DemoApp object"""
        self.init_gui()
        self.init_gst()

    def init_gui(self):
        """Initialize the GUI components"""
        self.window = gtk.Window()
        self.window.connect("delete-event", gtk.main_quit)
        self.window.set_default_size(400,200)
        self.window.set_border_width(10)
        vbox = gtk.VBox()
        self.textbuf = gtk.TextBuffer()
        self.text = gtk.TextView(self.textbuf)
        self.text.set_wrap_mode(gtk.WRAP_WORD)
        vbox.pack_start(self.text)
        self.button = gtk.ToggleButton("Speak")
        self.button.connect('clicked', self.button_clicked)
        vbox.pack_start(self.button, False, False, 5)
        self.window.add(vbox)
        self.window.show_all()

    def init_gst(self):
        """Initialize the speech components"""
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('partial_result', self.asr_partial_result)
        asr.connect('result', self.asr_result)
        asr.set_property('configured', True)
	asr.set_property('dsratio', 1)
	#AUTONOMOUS DICT	
#	asr.set_property('lm', '/home/eric/code/dev_stacks/cwru-ros-pkg/cwru_experimental/cwru_voice/model/1495.lm')
#       asr.set_property('dict', '/home/eric/code/dev_stacks/cwru-ros-pkg/cwru_experimental/cwru_voice/model/1495.dic')
	asr.set_property('lm', '/home/eric/code/dev_stacks/cwru-ros-pkg/cwru_experimental/cwru_voice/model/7023.lm')
        asr.set_property('dict', '/home/eric/code/dev_stacks/cwru-ros-pkg/cwru_experimental/cwru_voice/model/7023.dic')
	#MOTORIC
	#asr.set_property('lm', '/home/tony/Desktop/test/MOTORIC/5803.lm')
        #asr.set_property('dict', '/home/tony/Desktop/test/MOTORIC/5803.dic')
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::application', self.application_message)
        self.pipeline.set_state(gst.STATE_PAUSED)
	self.msg = String()
	self.msg2 = String()        
	self.pub = rospy.Publisher('chatter',String)
	self.pub2 = rospy.Publisher('speak',String)
	rospy.init_node('voice_cmd')
	


    def asr_partial_result(self, asr, text, uttid):
        """Forward partial result signals on the bus to the main thread."""
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """Forward result signals on the bus to the main thread."""
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """Receive application messages from the bus."""
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        elif msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])
            self.pipeline.set_state(gst.STATE_PAUSED)
            self.button.set_active(False)

    def partial_result(self, hyp, uttid):
        """Delete any previous selection, insert text and select it."""
        # All this stuff appears as one single action
        self.textbuf.begin_user_action()
        self.textbuf.delete_selection(True, self.text.get_editable())
        self.textbuf.insert_at_cursor(hyp)
        ins = self.textbuf.get_insert()
        iter = self.textbuf.get_iter_at_mark(ins)
        iter.backward_chars(len(hyp))
        self.textbuf.move_mark(ins, iter)
        self.textbuf.end_user_action()
	print "Partial: " + hyp
	hyp = hyp.lower()        
	if "stop" in hyp:
		str = "STOP!"
		print str		
		self.msg.data = "stop"
		rospy.loginfo(self.msg.data)
	        self.pub.publish(self.msg.data)
		
		self.msg2.data = str		
		rospy.loginfo(self.msg2.data)
	        self.pub2.publish(self.msg2.data)
		
    def final_result(self, hyp, uttid):
        """Insert the final result."""
        # All this stuff appears as one single action
        self.textbuf.begin_user_action()
        self.textbuf.delete_selection(True, self.text.get_editable())
        self.textbuf.insert_at_cursor(hyp)
        self.textbuf.end_user_action()
	print "Final: " + hyp
	hyp = hyp.lower()

	if "go to" and "bathroom" in hyp:
		str = "I understood. I am now going to the women's bathroom. \n"
		self.msg.data = "womens_bathroom"
	elif "go to" and "conference room" in hyp:
		str = "I am going to the conference room. \n"
		self.msg.data = "conference_room"
	elif "stop" in hyp:
		str = "STOP!"
		self.msg.data = "stop"
	else:
		print "I didn't understand. Please say a command. \n"
		self.msg.data = ""
		str = ""
	print str
	rospy.loginfo(self.msg.data)
        self.pub.publish(self.msg.data)

	self.msg2.data = str		
	rospy.loginfo(self.msg2.data)
        self.pub2.publish(self.msg2.data)



    def button_clicked(self, button):
        """Handle button presses."""
        if button.get_active():
            button.set_label("Stop")
            self.pipeline.set_state(gst.STATE_PLAYING)
        else:
            button.set_label("Speak")
            vader = self.pipeline.get_by_name('vad')
            vader.set_property('silent', True)

app = DemoApp()
gtk.main()
