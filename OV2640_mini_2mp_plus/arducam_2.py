from PIL import Image
import serial
import tkinter as tk

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

ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1.5)


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
                # print(data_in.__len__())
                # print('done')
                self.list_items[5].config(text="Bytes received = " + str(data_in.__len__()))
                break
        photo = open('testing_cam' + str(self.num) + '.jpg', 'wb')
        for i in range(data_in.__len__()):
            photo.write(data_in[i])
        photo.close()
        data_in.clear()
        img = Image.open('testing_cam' + str(self.num) + '.jpg')
        img.show()
        img.close()

    def change_q(self):
        qual = self.list_items[2].get()
        # print(list_quality[qual])
        self.list_items[4].config( text="Picture quality is " + self.list_qual[qual])
        qual += 2
        self.serial_in.write(str(qual).encode())


if __name__ == "__main__":
    cam_items = []
    cam = Cameras(0, ser, cam_items, list_quality)
    view = tk.Tk()
    view.geometry('280x75')
    view.title("ArduCAM")

    butt1 = tk.Button(view, text='Take photo', command=cam.take_photo)
    butt1.grid(row=0, column=0)
    cam_items.append(butt1) #0
    butt2 = tk.Button(view, text='Change quality', command=cam.change_q)
    butt2.grid(row=0, column=1)
    cam_items.append(butt2) #1
    droping = tk.IntVar()
    droping.set(2)
    cam_items.append(droping) #2
    options = [0, 1, 2, 3, 4, 5, 6, 7]
    drop = tk.OptionMenu(view, droping, *options)
    drop.grid(row=0, column=2)
    cam_items.append(drop) #3
    pic_lab = tk.Label(view, text="Picture quality is " + list_quality[droping.get()])
    pic_lab.grid(row=1, column=0, columnspan=3)
    cam_items.append(pic_lab) #4
    bit_lab = tk.Label(view, text="Bytes received = 0")
    bit_lab.grid(row=2, column=0, columnspan=3)
    cam_items.append(bit_lab) #5

    view.mainloop()
