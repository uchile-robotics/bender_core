from bender_utils.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from bender_utils import sh


class LaserCheckTask(SystemCheckTask):
    def __init__(self, _id):
        super(LaserCheckTask, self).__init__()
        self.id = _id
        self.symlink_base = "/dev/bender/sensors/"
        self.symlink_prefix = "hokuyo_"
        self.get_id_path = "/opt/bender/udev/getID"
        self.vendor_id = "15d1"  # hokuyo

    def check_id_exec(self):
        if not sh.file_exists(self.get_id_path):
            SystemCheck.print_warn("Program '{}' not found.".format(self.get_id_path), 1)
            return False
        return True

    def check(self):
        symlink = self.symlink_base + self.symlink_prefix + self.id
        SystemCheck.print_high("Target symlink  : " + symlink, 1)
        SystemCheck.print_high("Target laser id : " + self.id, 1)

        if not self.check_id_exec():
            SystemCheck.print_error("Will not proceed... Please install the required program.", 2)
            return False

        # path exists and symlink is not broken
        if sh.file_exists(symlink):

            SystemCheck.print_info("Hokuyo symlink exists ... ", 1)

            # valid id: ok
            cmd = '[ "$({} {} q)" = "{}" ]'.format(self.get_id_path, symlink, self.id)
            SystemCheck.print_info("Hokuyo id check...", 1)
            if sh.exec_cmd(cmd, level=1):
                SystemCheck.print_ok("The device is connected and working as desired.", 1)
                return True

            # invalid id: seek and repair
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
        devpaths = sh.get_devpath_list("/dev/ttyACM")
        if not devpaths:
            SystemCheck.print_error("Could not find any /dev/ttyACM[0-9] port. Is the laser connected?", 2)
            return False

        # check each /dev/ttyACM[0-9] port
        SystemCheck.print_info("Available ports (/dev/ttyACM[0-9]): " + str(devpaths), 1)
        for devpath in devpaths:
            SystemCheck.print_info("checking device at {} ... ".format(devpath), 2)

            # check vendor_id
            cmd = "[ $(udevadm info --name={}  | grep ID_VENDOR_ID | cut -d'=' -f2) = {} ]".format(devpath,
                                                                                                       self.vendor_id)
            SystemCheck.print_info("vendor_id check ...", 3)
            if not sh.exec_cmd(cmd, level=4):
                SystemCheck.print_info("invalid vendor_id ... ", 4)
                continue

            # check hokuyo_id
            cmd = '[ "$({} {} q)" = "{}" ]'.format(self.get_id_path, devpath, self.id)
            SystemCheck.print_info("Hokuyo id check ...", 3)
            if not sh.exec_cmd(cmd, level=4):
                SystemCheck.print_info("invalid hokuyo id ... ", 4)
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

        SystemCheck.print_error("No connected device matches hokuyo id {}. Is the laser connected?".format(self.id), 1)
        return False


def check_front():
    laser = SystemCheck("laser_front")
    laser.add_child(LaserCheckTask("H1311689"))
    return laser.check()


def check_rear():
    laser = SystemCheck("laser_rear")
    laser.add_child(LaserCheckTask("H0903381"))
    return laser.check()


def check_lasers():
    front_res = check_front()
    rear_res = check_rear()
    return front_res and rear_res
