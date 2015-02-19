import re
from scipy import io
import os
import rosbag

class ROSBagConverter:
    """ 
    A class to convert bag files to mat files.

    Notes
    -----
    If you are not comfortable with recursion, I would not recommend editing this code.
    I apologize for the rampant abuse of lists.
    
    """
    def __init__(self, path_to_bag_file, path_to_mat_directory):
        """
        Initialize an instance of ROSBagConverter

        Parameters
        ----------
        path_to_bag_file : str
            A path to an existing bag file.
        path_to_mat_directory : str
            A path to an existing directory where the .mat files will be written.

        Notes
        -----
        If a path is not valid for any reason, a ValueError exception will be raised.

        """
        self.mat_dict = dict()
        self.failed_file_list = []        
        self.bag_path = ""
        self.mat_path = ""
        self.bag_name = ""        
        self.bag_to_convert = None
        self.bag_iterator = None       #iterator returns a tuple of form (topic, message, time)   
        #Validate the path to the bag file
        if os.path.exists(path_to_bag_file):
            self.bag_path = path_to_bag_file
        else:
            raise ValueError("Path to Bag file does not exist. Please specify a valid path.")
        #Validate the path to the directory
        if os.path.isdir(path_to_mat_directory):
            self.mat_path = path_to_mat_directory
            #We will be writing files to this directory so make sure we have the trailing '/'
            if self.mat_path[-1] != "/":
                self.mat_path = self.mat_path + "/"
        else:
            raise ValueError("Path to mat directory is not valid. Please specify a valid path to a directory.")                                            
        #Parse the path for the bag name
        s = re.search(r'[^\/]*\.bag', self.bag_path)
        if s:
            self.bag_name = s.group()[:-4]    #Remove the .bag
        else:
            raise ValueError("Path to Bag file does not refer to a bag file. Please specify a path to an existing bag file.")
        try:
            self.bag_to_convert = rosbag.Bag(self.bag_path, 'r')    #Open the bag file for reading
        except rosbag.bag.ROSBagException, e:            
            raise ValueError("Specified file does not appear to be a bag file: {0}".format(str(e)))
        try:
            self.bag_iterator = self.bag_to_convert.read_messages()     #Generate an iterator for the bag file
        except rosbag.bag.ROSBagException, e:            
            raise ValueError("Specified bag file appears to be malformed: {0}".format(str(e)))

    def __iter__(self):
        return self.bag_iterator
                     
    def get_simple_type(self, bag_message):
        """
        Check to see if the obj is a parent to other objects until we get to an object with no children. 

        Parameters
        ----------
        bag_message : obj 
            An object with or without child objects.

        Returns
        -------
        A nested list with the childless objects.

        Notes
        -----
        Some objects use __dict__ instead of __slots__, which has yet to be checked in my code.  This affects the parsing of image bag files.       
     
        """
        #Check to see if the object has child objects
        if hasattr(bag_message, "__slots__"):
                result = []     #Create a list to store the childless objects
                for slot in bag_message.__slots__:      #For each child object
                        member = bag_message.__getattribute__(slot) 
                        result.append(self.get_simple_type(member))     #Check to see if it has child objects
                return result
        if isinstance(bag_message, tuple):
            if not bag_message:        #The mat filewriter doesn't like empty tuples
                return 0.0
        #The mat filewriter doesn't like strings due to some encoding issues I haven't figured out
        if isinstance(bag_message, str):
                return 0.0                  #For now, don't pass strings
        return bag_message
    
    def flatten_list(self, bag_list):
        """
        A generator function that flattens the passed object and yields an iterator whose individual elements can not be iterated over. 

        Parameters
        ----------
        bag_list : obj
            A parent object whose child objects have children, and both the parent and its children have defined __iter__ attributes.

        Returns
        -------
        An iterator containing every element in the bag_list object.

        Notes
        -----
        An example object is a list of lists.  Each list in the parent list has elements, so this function takes all of the elements from every list in the parent list and puts them into a single object.
        Graphically:
        Passing a list like: [[1,2],[1,3,[1,1,1,[1,1]]]]
        Yields an object with elements: 1,2,1,3,1,1,1,1,1
        
        """
        #Check to see if the object can be iterated over
        if hasattr(bag_list, "__iter__"):
            for member in bag_list:     #For every child object
                for element in self.flatten_list(member):   #Check to see if the child object is iterable and add to the generator
                    yield element
        else:
            yield bag_list
    #Yo dawg, I heard you like lists. So, I put lists in your lists, so your lists can list lists.

    def factor_header(self, bag_header):
        """
        A function that constructs header strings that show the object structure of the bag message data.

        Parameters
        ----------
        bag_header : a list returned from the get_header function
            The get_header function creates a list of lists; the bag_header argument is one of the child lists in the list of lists. 

        Returns
        -------
        A list of strings
    
        Notes
        -----
        Only use this function in conjunction with the get_header() function.  The get_header function is guaranteed to give parseable lists.
        Passing a list not created by the get_header function may result in an exception were the code tries to concatenate a string and a list.

        This function creates strings based on the explicit relationship between a parent object and its children.
        For example:
        An object defined in a bag message is of the form: ["header", ["seq", "stamp", ["secs, "nsecs"], "frame"]]
        We are assuming the list is of the form ["parent object", ["list of child objects"]]
        There are four total child data fields we are interested in: "seq", "secs", "nsecs", and "frame"
        We want to preserve the explicit parent-child relationship of these data fields, so the factor_header function returns:
        ["header.seq", "header.stamp.secs", "header.stamp.nsecs", "header.frame"]

        """
        if isinstance(bag_header, list):
            bag_header_string = bag_header[0] + "." #first item in the list must be a string; this object is the parent
            result = []
            for element in bag_header[1]:   #Parse the child list
                if isinstance(element, list):     #Check if the child list contains any lists
                    bag_header_list = self.factor_header(element)   #If so, factor the child list
                    for bag_item in bag_header_list:     #For every element in the factored child list
                        result.append(bag_header_string + bag_item)    #Add the parent string to the child string 
                else:
                    result.append(bag_header_string + element)  #If it is not a list, just add it to the parent string
            return result
        return bag_header

    def get_header(self, bag_message):
        """
        A function to create a data header to label the data in a mat file column.

        Parameters
        ----------
        bag_message : bag_message from a bag file
            A bag message generated from the bag_iterator
        
        Returns
        -------
        A list of lists

        Notes
        -----
        In order to be of any value, this function must be used in conjunction with the factor_header function.
        """
        if hasattr(bag_message, "__slots__"):   #Check to see if the object has child objects
            result = []
            for slot in bag_message.__slots__:      #For each child object
                member = bag_message.__getattribute__(slot)     #Get the child object
                field = self.get_header(member)     #Check to see if the child has children
                #We need to change the header to reflect all the child items in tuples
                if (isinstance(field, int) and (field > 0)):
                    for x in range(0, field):   #For every element in the tuple 
                        result.append(str(slot) + "." + str(x))     #Add the index of the element to the header string
                elif ((field != None) and (field != 0)):    
                    result.append([slot, field])    #Append the parent object and the returned child
                else:
                    result.append(slot)     #We found the last child; add it to the list
            return result
        else:
            if isinstance(bag_message, tuple):  #Tuples only have one header string for a tuple of n elements 
                return len(bag_message) #We need to make sure we have n header strings for a tuple with n elements

    def convert_bag_to_mat(self):
        """
        Parses a bag file, separates the bag file by topics, and writes a mat file or each topic.

        Returns
        -------
        1 : int
            A file failed to be written
        0 : int
            All files written successfully

        """
        for bag_topic, bag_msg, bag_time in self.bag_iterator:
            #Create a new dictionary for each new topic
            if bag_topic not in self.mat_dict:
                print "Parsing data for topic: {0}".format(bag_topic)                
                self.mat_dict[bag_topic] = dict()
                #Get the header sring for the topic
                mat_list = self.get_header(bag_msg)
                result = []
                for x in mat_list:
                    result.append(self.factor_header(x))    #Parse the header list of strings
                self.mat_dict[bag_topic]["header"] = list(self.flatten_list(result))    
                self.mat_dict[bag_topic]["data"] = []
            #Add the data from each bag message to the dictionary
            self.mat_dict[bag_topic]["data"].append(list(self.flatten_list(self.get_simple_type(bag_msg))))     
        #Call the filewriter
        self.write_mat_file()
        if not self.failed_file_list:
            return 0
        print "Failed to write the following mat files:"
        for x in self.failed_file_list:
            print str(x)
        return 1

    def write_mat_file(self):
        """
        Calls the mat filewriter, which writes a mat file for each topic in the bag file.
        
        Returns
        -------
        None
    
        Notes
        -----
        While the function doesn't explicitly return anything, it does:
        Create a mat file per topic in the specified directory
        Update self.failed_file_list for any files that failed to be written

        Notes on the mat filewriter
        ---------------------------
        The mat filewriter is a fickle beast.
        It particularly hates:
        Nested dictionaries
        Sequences
        Certain complex structures
        Structures with strings
        Anything not UTF-8 encoded
        Puppies 
        Ice cream
        That warm, fuzzy feeling you get while holding someone you love

        I would recommend writing a new one, time permitting        
        """
        for topic in self.mat_dict:
            print "Writing data to mat file for topic: {0}".format(topic)
            named_topic = topic.replace("/", "_")   #Don't want to go to a non-existent directory.
            #Check to make sure the header exactly matches the data
            if (len(self.mat_dict[topic]["header"]) != len(self.mat_dict[topic]["data"][0])):
                print "ERROR: Header doesn't match up to data fields."
                self.failed_file_list.append(named_topic + ".mat")
                continue
            #Start writing data to a .mat file
            try:            
                io.savemat((self.mat_path + self.bag_name + "_" + named_topic), self.mat_dict[topic])
            #Check to see if the filewriter refuses to write a type/structure to the file.
            except ValueError, e:   #Filewriter doesn't like one of our structures.
                print "Invalid structure in dictionary: {0}".format(str(e))
                #Clean-up the created file.
                os.remove(self.mat_path + self.bag_name + "_" + named_topic + ".mat")
                #Log the failure.
                self.failed_file_list.append(named_topic + ".mat")
                continue    #Write the next topic in the dictionary.
            except TypeError, e:    #Filewriter doesn't like a certain type.
                print "Invalid type in dictionary: {0}".format(str(e))
                #Clean-up the created file.
                os.remove(self.mat_path + self.bag_name + "_" + named_topic + ".mat")
                #Log the failure.
                self.failed_file_list.append(named_topic + ".mat")
                continue    #Write the next topic in the dictionary


