from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from uchile_util import sh


class BaseCheckTask(SystemCheckTask):
    def __init__(self):
        super(BaseCheckTask, self).__init__()
        self.symlink_base = "/dev/bender/"
        self.symlink_name = "base"
        self.vendor_id = "067b"

    def check(self):
        symlink = self.symlink_base + self.symlink_name
        SystemCheck.print_high("Target symlink  : " + symlink, 1)

        # path exists and symlink is not broken
        if sh.file_exists(symlink):

            SystemCheck.print_info("Pioneer symlink exists ... ", 1)

            # check vendor_id
            cmd = "[ $(udevadm info --name={}  | grep ID_VENDOR_ID | cut -d'=' -f2) = {} ]".format(symlink,
                                                                                                       self.vendor_id)
            SystemCheck.print_info("vendor_id check ...", 1)
            if sh.exec_cmd(cmd, level=1):
                # another check?
                #

                SystemCheck.print_ok("The device is connected and working as desired.", 1)
                return True

            # invalid vendor id: seek and repair
            SystemCheck.print_warn("Path {} exists, but is assigned to an invalid device ...".format(symlink), 1)

        else:
            # link not found: seek
            SystemCheck.print_warn("Symlink to the device not found or broken ... ", 1)

        # delete possible invalid symlink
        if sh.file_exists_strict(symlink):
            SystemCheck.print_info("Deleting invalid symlink ... ", 2)
            if not sh.delete_file(symlink, level=2, use_sudo=True):
                SystemCheck.print_error("Failed to delete symlink.", 2)
                return False

        SystemCheck.print_info("Looking for devices on the system ... ", 1)

        # display available ports
        devpaths = sh.get_devpath_list("/dev/ttyUSB")
        if not devpaths:
            SystemCheck.print_error("Could not find any /dev/ttyUSB[0-9] port. Is the base connected?", 2)
            return False

        # check each /dev/ttyACM[0-9] port
        SystemCheck.print_info("Available ports (/dev/ttyUSB[0-9]): " + str(devpaths), 1)
        for devpath in devpaths:
            SystemCheck.print_info("checking device at {} ... ".format(devpath), 2)

            # check vendor_id
            cmd = "[ $(udevadm info --name={}  | grep ID_VENDOR_ID | cut -d'=' -f2) = {} ]".format(devpath,
                                                                                                       self.vendor_id)
            SystemCheck.print_info("vendor_id check ...", 3)
            if not sh.exec_cmd(cmd, level=4):
                SystemCheck.print_info("invalid vendor_id ... ", 4)
                continue

            # create symlink
            SystemCheck.print_high(
                "Devpath {} matches target id {}. Creating symlink to {} ...".format(devpath, self.id, symlink), 3)

            if not sh.mkdir_p(self.symlink_base, level=3, use_sudo=True):
                SystemCheck.print_error("Failed to create directory: {}".format(self.symlink_base), 3)
                return False

            if not sh.create_symlink(devpath, symlink, level=3, use_sudo=True):
                SystemCheck.print_error("Failed to create symlink {} -> {}".format(devpath, symlink), 3)
                return False

            # OK!
            SystemCheck.print_ok("Successfully assigned symlink. Device is ready to use.", 3)
            return True

        SystemCheck.print_error(
            "No connected device matches pioneer p3at vendor_id {}. Is the laser connected?".format(self.vendor_id), 1)
        return False


def check():
    base = SystemCheck("base")
    base.add_child(BaseCheckTask())
    return base.check()
