from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from uchile_util import sh


class CameraCheckTask(SystemCheckTask):
    def __init__(self, linkname, id, vendor_id):
        super(CameraCheckTask, self).__init__()
        self.id = id  # ID_SERIAL
        self.linkname = linkname
        self.symlink_prefix = "/dev/bender/sensors/"
        self.vendor_id = vendor_id

    def check_vendor_id(self, path, id, level):

        # check vendor_id
        cmd = "[ \"$(udevadm info --name={}  | grep ID_VENDOR_ID= | cut -d'=' -f2)\" = \"{}\" ]".format(path, id)
        SystemCheck.print_info("vendor_id check ...", level)
        if not sh.exec_cmd(cmd, level+1):
            SystemCheck.print_info("invalid vendor_id ... ", level + 1)
            return False
        return True

    def check_device_id(self, path, id, level):

        # check vendor_id
        cmd = "[ \"$(udevadm info --name={}  | grep ID_SERIAL= | cut -d'=' -f2)\" = \"{}\" ]".format(path, id)
        SystemCheck.print_info("short serial id check ...", level)
        if not sh.exec_cmd(cmd, level+1):
            SystemCheck.print_info("invalid serial_id ... ", level + 1)
            return False
        return True

    def check(self):
        symlink = self.symlink_prefix + self.linkname
        SystemCheck.print_high("Target symlink  : " + symlink, 1)
        SystemCheck.print_high("Target camera id : " + self.id, 1)

        # path exists and symlink is not broken
        if sh.file_exists(symlink):

            SystemCheck.print_info("Camera symlink exists ... ", 1)

            # valid vendor id: ok
            if self.check_vendor_id(symlink, self.vendor_id, 1) and self.check_device_id(symlink, self.id, 1):
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
        devpaths = sh.get_devpath_list("/dev/video")
        if not devpaths:
            SystemCheck.print_error("Could not find any /dev/video[0-9] port. Is the camera connected?", 2)
            return False

        # check each /dev/video[0-9] port
        SystemCheck.print_info("Available ports (/dev/video[0-9]): " + str(devpaths), 1)
        for devpath in devpaths:
            SystemCheck.print_info("checking device at {} ... ".format(devpath), 2)

            # valid vendor id: ok
            if not self.check_vendor_id(devpath, self.vendor_id, 3):
                continue

            # valid device serial number
            if not self.check_device_id(devpath, self.id, 3):
                continue

            # create symlink
            SystemCheck.print_high(
                "Devpath {} matches target id {}. Creating symlink to {} ...".format(devpath, self.id, symlink), 3)

            if not sh.mkdir_p(self.symlink_prefix, level=3, use_sudo=True):
                SystemCheck.print_error("Failed to create directory: {}".format(self.symlink_prefix), 3)
                return False

            if not sh.create_symlink(devpath, symlink, level=3, use_sudo=True):
                SystemCheck.print_error("Failed to create symlink {} -> {}".format(devpath, symlink), 3)
                return False

            # OK!
            SystemCheck.print_ok("Successfully assigned symlink. Device is ready to use.", 3)
            return True

        SystemCheck.print_error("No connected device matches camera id {}. Is the camera connected?".format(self.id), 1)
        return False


def check_selfiecam():
    cam = SystemCheck("selfie camera")
    cam.add_child(CameraCheckTask("selfiecam", "046d_0825_271BA220", "046d"))
    # cam.add_child(CameraCheckTask("selfiecam", "093a_2622", "093a"))
    return cam.check()


def check_cameras():
    return check_selfiecam()
