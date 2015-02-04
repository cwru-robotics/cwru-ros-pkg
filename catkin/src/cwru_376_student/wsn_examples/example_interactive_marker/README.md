# example_interactive_marker

The example code here illustrates some visualization features with markers.

marker_example.cpp shows has hard-coded marker coordinates and hard-coded marker parameters.  It places a "wall" of small spheres in front of Atlas.
To see the effect, run: `rosrun example_interactive_marker marker_example`. In rviz, "add" a "Marker" to the displays, and set the topic to:
"wsn_marker." You should see a wall of small, red spheres grow in a raster pattern.

A second example is a pair: a listener and a publisher.  The listener "marker_listener.cpp" listens for "points" on the topic "marker_listener."
As each message arrives, the callback function adds the received point to the marker list and republishes this for rviz display.

The complementary node is: example_marker_user_app.cpp.  This node generates the same raster pattern of small spheres within a plane, and it
publishes each new marker coordinate to the topic "marker_listener."  The example app may be emulated to include its behavior within another program.
E.g., one may compute points of interest within a program (such as reachability indications), and tell "marker_listener" to display each such point in rviz.

To see these work, run both nodes and add a Marker to the rviz display with the topic "marker_publisher".

The example "IM_example" is more complex.  It illustrates how to create an "interactive marker."  This example uses red, green and blue
arrows in a triad to illustrate a frame orientation.  The composite marker can be moved interactively
in 6-D.  Resulting marker pose is published within a message of type "visualization_messages/InteractiveMarkerFeedback" on the topic "/example_marker/feedback."  
Details are explained in the embedded comments and in the document "Interactive_markers" within this repository's "documents/class_docs_2014" directory.


    
