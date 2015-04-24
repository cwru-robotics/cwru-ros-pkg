Example (quick hack) GUI for (hard-coded) service calls:

Here is a tool to streamline some of our robot interfacing.

#Running this program:

If you cd to this directory (wsn_examples/service_call_gui), then (from a terminal), run: 
`./service_call_gui`

A GUI will pop up with 6 buttons.

The first button (move_trigger) runs: rosservice call move_trigger 1

The next button--set mode 0-- runs: rosservice call process_mode 0

The next several buttons do: rosservice call process_mode 1  (through mode 4).

#Creating this program:

This quick hack was developed using "glade" (http://glade.gnome.org) to design the user interface.
This interface links button actions to callback functions.  

Using this application (alternatively, other applications), the result is a user interface file, in this case, "service_btns.glade".  This is an XML file, and it can be edited with a text editor, if desired (though using the Glade application is easier).

Within the UI file, service_btns.glade, there is a description of graphical appearances and mappings from events, such as button clicks, to named callback functions.

The callback functions are implemented in another file: service_call_gui.c.  For example, the a function "move_trigger_cb()", which is named in the glade (XML) file, is the callback function associated with clicking the button labelled "move_trigger."  

In this crude example, callback functions within service_call_gui.c contain "system" calls, e.g.:
system("rosservice call move_trigger 1");

Clicking the button "move_trigger" invokes the callback function "move_trigger_cb()", which invokes a system call that does the same thing as typing in: rosservice call process_mode 0
at a command line.

The application is compiled with:

gcc -o service_call_gui service_call_gui.c $(pkg-config --cflags --libs gtk+-2.0 gmodule-2.0)

(which requires installing the referenced libraries).

More generally, a catkin build of this code would allow creating a better integration with ROS.

Nonetheless, this crude example should be handy for graphically invoking these pre-defined service calls.
