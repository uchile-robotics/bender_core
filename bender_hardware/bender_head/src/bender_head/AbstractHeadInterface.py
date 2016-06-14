#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# mpavez

class AbstractHeadInterface(object):
    """
    Abstract class for implementing an interface to head hardware
    All interfaces must inherit from this class.

    Subclasses must implement the following methods:
    - connect()         : connects to head hardware
    - is_connected()    : checks for connectivity
    - close()           : 
    - loop()            : 
    - write(self, key,(): 
    
    """

    def __init__(self):
        return


    def connect(self):
        """
        Abstract method you must supply when implementing a HeadInterface.

        It attempts to stablish a connection to the head hardware
        returns nothing
        """
        raise NotImplementedError("HeadInterface %s doesn't implement connect()" % (self.__class__.__name__))


    def is_connected(self):
        """
        Abstract method you must supply when implementing a HeadInterface.

        returns True if the connection is currently active and works!
        returns False otherwise
        """
        raise NotImplementedError("HeadInterface %s doesn't implement is_connected()" % (self.__class__.__name__))


    def close(self):
        """
        Abstract method you must supply when implementing a HeadInterface.

        Closes the connection to head hardware
        returns nothing
        """
        raise NotImplementedError("HeadInterface %s doesn't implement close()" % (self.__class__.__name__))



    def write(self, key, value):
        """
        Abstract method you must supply when implementing a HeadInterface.

        Sends a (key, value) pair to the head hardware. Both key, value are Strings.
        - key   examples are: "movex", "mouth", "emotion"
        - value examples are: "20"   , "off"  , "happy"

        returns nothing
        """
        raise NotImplementedError("HeadInterface %s doesn't implement write()" % (self.__class__.__name__))


    def loop(self):
        """
        Abstract method you must supply when implementing a HeadInterface.

        It attempts to reconnect if neccessary and then sends queued messages.
        There can be queued messages if the head was disconnected previously, so
        this is a way to recover the its desired state.
        """
        raise NotImplementedError("HeadInterface %s doesn't implement loop()" % (self.__class__.__name__))

