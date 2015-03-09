# ROS raw_message
The ROS raw_message library provides means to subscribe to ROS nodes at run-time.
This makes it possible to let a node, written in C++, subscribe to arbitrary topics, which are not (exactly) known during compilation of the node.
Similar as several ROS tools are capable of, but either using Python, or by using their own (inaccessible) implementations.

The raw_message library makes use of the [ShapeShifter](http://docs.ros.org/indigo/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html) class, provided by the [topic_tools](http://wiki.ros.org/topic_tools) package.
The library has a MessageDecoder class that is able to decode the raw, binary blob that is received the topic and provide information about the available fields and their values.
Basically, it makes this (undocumented) API more and generically accessible.


## Limitation
Currently, it is only possible to read from arbitrary topics, writing to them is not *(yet)* supported.
