#!/usr/bin/env python
"""
Samuel Barham and Matthew Goldberg

The script is divided into two major sections -- the first contains the
parser logic, the second contains the code for subscribing to the
'language_input' node. It made sense to separate and abstract these two
components as much as possible.

Note: we subscribe to the 'language_input' topic, which is the topic to
which all language input devices (read 'nodes') should publish their data.
Typically, this will be the the 'speech_recognizer' node, but in other cases,
we may wish to simplify the language interface -- removing the inconsistency
of imperfect speech recognition -- by using console input.
"""

import rospy
#import std_msgs.msg.String
import std_msgs
import re

"""
PARSING LOGIC
"""

recognized = ['person', 'people', 'bird', 'cat', 'cow', 'dog', 'horse', 'sheep',
'airplane', 'bicycle', 'boat', 'bus', 'car', 'motorcycle', 'train', 'bottle',
'chair', 'dining table', 'potted plant', 'sofa', 'tv', 'monitor']

def parse(input):
    input = str(input)[5:]
    print "Received string: " + input
    input = input.lower()
    thing_to_look_for, predicate = None, None
    regex = '(how(\(\d+\))? many(\(\d+\))? ([a-z]+)s)|(is(\(\d+\))? ' \
'there(\(\d+\))? a(\(\d+\))? ([a-z]+)(\(\d+\))?)|(see(\(\d+\))? a(\(\d+\))? ' \
'([a-z]+)(\(\d+\))?)'
    r = re.compile(regex)
    m = r.search(input)
    if m is not None:
        if m.group(4):
            thing_to_look_for = m.group(4)
            predicate = 'count'
        if m.group(9):
            thing_to_look_for = m.group(9)
            predicate = 'exists'
        if m.group(14):
            thing_to_look_for = m.group(14)
            predicate = 'exists'
    else if input is not "":
        predicate = 'error'
        print "I didn't recognize a request"

    if thing_to_look_for:
        print "You asked about recognizing: " + thing_to_look_for
        if thing_to_look_for in recognized:
            res = predicate + '(' + thing_to_look_for + ')'
            publisher.publish(res)
            #print "Publishing predicate query: " + res
        else:
            print "I can't recognize a(n): " + thing_to_look_for

"""
ROSPY SUBSCRIPTION INTERFACE
"""

publisher = rospy.Publisher("language", std_msgs.msg.String, queue_size=5)

def listener():
    rospy.init_node("simple_parser")
    rospy.Subscriber("language_input", std_msgs.msg.String, callback)
    rospy.spin()

def callback(input_msg):
    meaning = parse(input_msg)
    publisher.publish(meaning)

if __name__ == "__main__":
    listener()
