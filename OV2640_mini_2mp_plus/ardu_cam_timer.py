
import serial
import time
import datetime

list_quality = [
    "160x120",
    "176x144",
    "320x240",
    "352x288",
    "640x480",
    "800x600",
    "1024x768",
    "1280x1024"
]

item_list = []
ser1 = serial.Serial('/dev/ttyUSB0', 921600, timeout=1.5)


class Cameras:
    def __init__(self, num, serial_in, list_items, list_qual):
        self.num = num
        self.serial_in = serial_in
        self.list_items = list_items
        self.list_qual = list_qual

    def take_photo(self):
        data_in = []
        self.serial_in.write(b'1')
        while self.serial_in.inWaiting() == 0:
            pass
        while True:
            try:
                data_in.append(int(self.serial_in.read(1)[-1]).to_bytes(1, 'little'))
            except:
                print("Bytes received = " + str(data_in.__len__()))
                break
        stamp = str(datetime.datetime.time(datetime.datetime.today()))
        stamp = str(stamp[0:2] + "_" + stamp[3:5] + "_" + stamp[6:8])
        photo = open('photos/testing_' + stamp + '.jpg', 'wb')
        for i in range(data_in.__len__()):
            photo.write(data_in[i])
        photo.close()
        data_in.clear()

    def change_q(self, qual_num):
        qual = qual_num + 2
        self.serial_in.write(str(qual).encode())


if __name__ == "__main__":
    cam1 = Cameras(0, ser1, item_list, list_quality)
    cam1.change_q(7)
    time.sleep(3)
    while True:
        cam1.take_photo()
        for i in range(30):
            time.sleep(60)
            print(i+1)
