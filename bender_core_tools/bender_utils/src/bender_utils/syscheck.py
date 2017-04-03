#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""This module implements Hardware check methods"""

from __future__ import print_function
import os
import sys
from termcolor import cprint


class SystemCheckTask(object):
    def __init__(self):
        pass

    def check(self):
        raise NotImplementedError("Class %s doesn't implement check()" % self.__class__.__name__)


def padding(level=0):
    if level == 0:
        return ""
    return '   ' * level + "- "


class SystemCheck(object):
    def __init__(self, shortname):
        self.children = list()
        self.shortname = shortname

    def add_child(self, child):
        if not issubclass(type(child), SystemCheckTask):
            SystemCheck.print_error("{} is not derived class from SystemCheckTask".format(child.__class__.__name__))
            return
        self.children.append(child)

    def check_children(self):
        complete_result = True
        if not self.children:
            SystemCheck.print_warn("There are no tasks to check", 1)
            return False
        for child in self.children:
            result = child.check()
            complete_result = complete_result and result
        return complete_result

    def check(self):
        SystemCheck.print_title("Check Target: '{}'".format(self.shortname))
        return self.check_children()

    @staticmethod
    def print_title(data):
        cprint("\n" + data, 'white', attrs=['bold'])

    @staticmethod
    def print_high(data, level=0):
        cprint(padding(level) + data, 'blue')

    @staticmethod
    def print_cmd(data, level=0):
        cprint(padding(level) + " $ " + data, 'cyan')

    @staticmethod
    def print_ok(data, level=0):
        cprint(padding(level) + "[ OK ]  " + data, 'green')

    @staticmethod
    def print_info(data, level=0):
        cprint(padding(level) + data)

    @staticmethod
    def print_warn(data, level=0):
        cprint(padding(level) + "[WARN]  " + data, 'yellow')

    @staticmethod
    def print_error(data, level=0):
        cprint(padding(level) + "[FAIL]  " + data, 'red')

    @staticmethod
    def exit_error():
        sys.exit(-1)

    @staticmethod
    def exit_ok():
        sys.exit(0)

    @staticmethod
    def exit(ok=False):
        if not ok:
            SystemCheck.exit_error()
        else:
            SystemCheck.exit_ok()


class FileCheckTask(SystemCheckTask):
    def __init__(self, filename):
        super(FileCheckTask, self).__init__()
        self.filename = filename

    def check(self):
        result = bool(os.path.exists(self.filename))
        if result:
            SystemCheck.print_ok('port found: {}'.format(self.filename), 1)
        else:
            SystemCheck.print_error('port not found: {}'.format(self.filename), 1)
        return result
