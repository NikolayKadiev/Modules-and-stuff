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

def take_photo():
    data_in = []
    ser.write(b'1')
    while ser.inWaiting() == 0:
        pass
    while True:
        try:
            data_in.append(int(ser.read(1)[-1]).to_bytes(1, 'little'))
        except:
            # print(data_in.__len__())
            # print('done')
            bit_lab.config(text="Bytes received = " + str(data_in.__len__()))
            break
    photo = open('testing.jpg', 'wb')
    for i in range(data_in.__len__()):
        photo.write(data_in[i])
    photo.close()
    data_in.clear()
    img = Image.open('testing.jpg')
    img.show()
    img.close()


def change_q():
    qual = droping.get()
    # print(list_quality[qual])
    pic_lab.config( text="Picture quality is " + list_quality[qual])
    qual += 2
    ser.write(str(qual).encode())


if __name__ == "__main__":
    view = tk.Tk()
    view.geometry('280x75')
    view.title("ArduCAM")

    butt1 = tk.Button(view, text='Take photo', command=take_photo)
    butt1.grid(row=0, column=0)
    butt2 = tk.Button(view, text='Change quality', command=change_q)
    butt2.grid(row=0, column=1)
    droping = tk.IntVar()
    droping.set(2)
    options = [0, 1, 2, 3, 4, 5, 6, 7]
    drop = tk.OptionMenu(view, droping, *options)
    drop.grid(row=0, column=2)
    pic_lab = tk.Label(view, text="Picture quality is " + list_quality[droping.get()])
    pic_lab.grid(row=1, column=0, columnspan=3)
    bit_lab = tk.Label(view, text="Bytes received = 0")
    bit_lab.grid(row=2, column=0, columnspan=3)

    view.mainloop()