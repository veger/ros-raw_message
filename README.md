# ROS raw_message
The ROS raw_message library provides means to subscribe to ROS nodes at run-time.
This makes it possible to let a node, written in C++, subscribe to arbitrary topics, which are not (exactly) known during compilation of the node.
Similar as several ROS tools are capable of, but either using Python, or by using their own (inaccessible) implementations.

## MessageDecoder
The library has a MessageDecoder class that is able to decode the raw, binary blob that is received the topic and provide information about the available fields and their values.
It makes makes use of the [ShapeShifter](http://docs.ros.org/indigo/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html) class, provided by the [topic_tools](http://wiki.ros.org/topic_tools) package.
Basically, it makes this (undocumented) API more and generically accessible.

## MessagePublisher
The MessagePublisher class is able to send data on topic without the limitation of strict compile-time message type, just instantion the class using one of the supported FieldTypes.
This class solves a less complex problem compared to the MessageDecoder class, but is added to be an 'opposite' for the MessageDecoder, so this library is (more) complete.

## License
The ROS raw_message library licenced under [GNU GPLv3](LICENSE).
