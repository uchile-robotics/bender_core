import os
import subprocess
from syscheck import SystemCheck


def get_devices_by_vendor(vendor_id):
    """
    returns a list of usb.core.Device objects matching vendor_id
    """
    # this requires to install
    import usb.core
    return list(usb.core.find(idVendor=vendor_id, find_all=True))


def get_devpath_list(path):
    """
    returns a list of valid ports following the pattern: path[0-9]
    """
    result = []
    for i in range(10):
        port = path + str(i)
        if file_exists(port):
            result.append(port)
    return result


def exec_cmd(cmd, level=0, use_sudo=False):
    if use_sudo:
        cmd = "/usr/bin/sudo " + cmd
    try:
        if level >= 0:
            SystemCheck.print_cmd(cmd, level)
        return subprocess.call(cmd, shell=True) == 0
    except OSError:
        return False


def delete_file(filename, level=0, use_sudo=False):
    cmd = "rm -f {}".format(filename)
    return exec_cmd(cmd, level, use_sudo)


def mkdir_p(path, level=0, use_sudo=False):
    cmd = "mkdir -p {}".format(path)
    return exec_cmd(cmd, level, use_sudo)


def create_symlink(source, symlink, level=0, use_sudo=False):
    cmd = "ln -s {} {}".format(source, symlink)
    return exec_cmd(cmd, level, use_sudo)


def file_exists_strict(filename):
    """
    return True if path refers to an existing path, even if symlink is broken
    """
    cmd = "[ -e {} ]".format(filename)
    return exec_cmd(cmd, -1, False)


def file_exists(filename):
    """
    return True if path refers to an existing path.
    returns False for broken symbolic links.
    """
    return os.path.exists(filename)
