import sensor, image, time
from pyb import UART

uart = UART(3, 19200)


thresholds1 = [(15, 45, 15, 50, 1, 30)] # Red cube threshold

thresholds2 = [(11, 71, -128, -24, 22, 127)]# Green cube threshold

MinMaxX = 80
DistM = 550


sensor.reset()                      # Reset and initialize the sensor.
sensor.set_vflip(0)
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.


def uSent(typ,x,dist):
    buff = 0
    buff |= typ<<6
    buff |= x << 4
    buff |= dist
    print(bin(buff))
    uart.writechar(buff)


def map( x,  in_min,  in_max,  out_min,  out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min




while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image
    surfGreen = 0
    xGreen = 0
    surfRed = 0
    xRed = 0


    for blob in img.find_blobs(thresholds1, pixels_threshold=10, area_threshold=45, merge=True, margin=10):
        if blob == None: # If there is no red block
            img.draw_line(80,0,80,120,color=(250,255,255))
            continue
        else:
          img.draw_line(80,0,80,120,color=(250,255,255))
          img.draw_rectangle(blob.rect(), color=(255,0,0)) # draw rect around the Red blob
          img.draw_cross(blob.cx(), blob.cy(), color=(255,255,0)) # draw cross in the center of the blob
          img.draw_string(blob.x(), blob.y(), str(blob.area()), color=(0, 255, 255)) # print blob's size in the left-top corner

          xx = blob.cx()
          xGreen = xx
          dist = xx - 160 # Find distance to the senter
          surfGreen = blob.w()*blob.h()



    for blob in img.find_blobs(thresholds2, pixels_threshold=5, area_threshold=45, merge=True, margin=10):
        if blob == None:# If there is no green block
            img.draw_line(80,0,80,120,color=(250,255,255))
            continue
        else:
          img.draw_line(80,0,80,120,color=(250,255,255))
          img.draw_rectangle(blob.rect(), color=(0,255,0)) # draw rect around the blob
          img.draw_cross(blob.cx(), blob.cy(), color=(255,255,0)) # draw cross in the center of the blob
          img.draw_string(blob.x(), blob.y(), str(blob.area()), color=(0, 255, 255))# print blob's size in the left-top corner
          xx = blob.cx()
          xRed = xx
          dist = xx - 160# Find distance to the senter
          surfRed = blob.w()*blob.h()

    if(xGreen > MinMaxX+10):
        xGreen = 2
    elif(xGreen < MinMaxX-10):
        xGreen = 0
    else:
        xGreen = 1

    if(xRed > MinMaxX+10):
        xRed = 2
    elif(xRed < MinMaxX-10):
        xRed = 0
    else:
        xRed = 1


    if(surfGreen > surfRed):
        uSent(0, xGreen,int(map(surfGreen,45,10000,0,15)))

    elif(surfGreen < surfRed):
         uSent(1, xRed, int(map(surfRed,45,10000,0,15)))
    elif surfGreen  == 0 and surfRed == 0:
         continue
