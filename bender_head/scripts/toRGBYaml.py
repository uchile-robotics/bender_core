#!/usr/bin/python

__author__ = 'gdiaz'

class toRGBYaml(object):
    def __init__(self):
        # self.data_file = open("colors.txt", "r")
        # self.out_file = open("rgb_colors.yaml", "wb+")
        self.data_file2 = open("colors2.txt", "r")
        self.out_file2 = open("rgb_colors2.yaml", "wb+")

    def getInfo(self, line):
        i = 0
        for char in line:
            if char == "\t":
                color_name = line[0:i]
                r = line[i+10:i+13]
                g = line[i+14:i+17]
                b = line[i+18:i+21]
                return [color_name, r, g, b]
            i += 1

    def getInfo2(self, line):
        i = 0
        for char in line:
            if char == "\t": #end number, next to this will be the name and code
                sub_line = line[i+1:-1]
                print sub_line
                j=0
                for sub_char in sub_line:
                    if sub_char == "\t":
                        color_name = sub_line[0:j]
                        r_hex = sub_line[j+2:j+4]
                        g_hex = sub_line[j+4:j+6]
                        b_hex = sub_line[j+6:j+8]
                        r = str(int(r_hex,16))
                        g = str(int(g_hex, 16))
                        b = str(int(b_hex, 16))
                        return [color_name, r, g, b]
                    j+=1
            i += 1

    def convert1(self, file_size):
        self.out_file.write("#To see refence colors go to: https://en.wikipedia.org/wiki/Web_colors\n")
        self.out_file.write("eye_colors:\n")
        for i in range(file_size):
            line = self.data_file.readline()
            if line[0] == "#":
                self.out_file.write("  "+line+"\n")
            else:
                info = self.getInfo(line)
                color_name = info[0]
                r = info[1]
                g = info[2]
                b = info[3]
                formated_line = "  "+color_name+" : ["+r+","+g+","+b+"]"
                self.out_file.write(formated_line+"\n")
        self.data_file.close()
        self.out_file.close()

    def convert2(self, file_size):
        self.out_file2.write("#To see refence colors go to: https://jonasjacek.github.io/colors/\n")
        self.out_file2.write("eye_colors:\n")
        for i in range(file_size):
            line = self.data_file2.readline()
            info = self.getInfo2(line)
            color_name = info[0]
            r = info[1]
            g = info[2]
            b = info[3]
            formated_line = "  "+color_name+" : ["+r+","+g+","+b+"]"
            self.out_file2.write(formated_line+"\n")
        self.data_file2.close()
        self.out_file2.close()

    def convert(self):
        pass

if __name__ == '__main__':
    format_converter = toRGBYaml()
    #format_converter.convert1(151)
    format_converter.convert2(256)