import sensor, image, time
from pyb import SPI

spi = SPI(2, SPI.PERIPHERAL, baudrate=600000, polarity=1, phase=0, crc=0x7)

'''
SPI pins are: (SCK, MISO, MOSI) = (P2, P1, P0) = (PB13, PB14, PB15)
Which is used is determined by the number 1 or 2 in SPI setup(above)


spi.send(dist*1000+1)


How to decrypt:
    Last number is type of Block
    1 == Red color
    2 == Green color

    Before the Color number(above) goes "00"

    The first number(can be multidigit) is distance in pixels
'''

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
#sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.



thresholds1 = [(15, 45, 15, 50, 1, 30)] # Red cube threshold

thresholds2 = [(15, 44, -40, -18, 1, 31)]# Green cube threshold

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image


    for blob in img.find_blobs(thresholds1, pixels_threshold=10, area_threshold=250, merge=True, margin=10,roi=(4,80,314,160) ):
        if blob == None: # If there is no red block
            img.draw_line(160,0,160,240,color = (255,255,255))# Draw a line at the screen senter
            continue
        else:
          img.draw_rectangle(blob.rect(), color=(255,0,0)) # draw rect around the Red blob
          img.draw_cross(blob.cx(), blob.cy(), color=(255,255,0)) # draw cross in the center of the blob
          img.draw_string(blob.x(), blob.y(), str(blob.area()), color=(0, 255, 255)) # print blob's size in the left-top corner
          img.draw_line(160,0,160  ,240,color = (255,255,255))# Draw a line at the screen senter
          img.draw_line(blob.cx(),blob.cy(),160,blob.cy(),color = (100,100,100))# Draw a line to the screen senter
          xx = blob.cx()
          dist = xx - 160 # Find distance to the senter
          spi.send(dist*1000+1)# send 3 bytes


    for blob in img.find_blobs(thresholds2, pixels_threshold=20, area_threshold=250, merge=True, margin=10,roi=(0,80,320,160)):
        if blob == None:# If there is no green block
            img.draw_line(160,0,160,240,color = (255,255,255))# Draw a line at the screen senter
            continue
        else:
          img.draw_rectangle(blob.rect(), color=(0,255,0)) # draw rect around the blob
          img.draw_cross(blob.cx(), blob.cy(), color=(255,255,0)) # draw cross in the center of the blob
          img.draw_string(blob.x(), blob.y(), str(blob.area()), color=(0, 255, 255))# print blob's size in the left-top corner
          img.draw_line(160,0,160  ,240,color = (255,255,255))# Draw a line at the screen senter
          img.draw_line(blob.cx(),blob.cy(),160,blob.cy(),color = (150,255,255))# Draw a line to the screen senter
          xx = blob.cx()
          dist = xx - 160# Find distance to the senter
          spi.send(dist*1000+2)# send 3 bytes
