# esp32CamWithMotionDetection

Working towards a lower power 'security' camera based on esp32 cam. Motion detection is based on frame differencing rather than PIR (or simular). 
Motion detection takes place as follows

JPEG->GreyScale->GaussianBlur->Diff with last frame->Threshold->erode the threshold. 

The number of pixels on the eroded image are counted. If the sum is above some threshold, movement is assumed.

The camera is intended on use in a vegitated area, which makes the motion detection susceptible to false positives in windy conditions. To counter
this, the Eroded image is dilated and added to an ignore mask, this mask is aged, so areas in which frame with frequent movement are ignored, while allowing
motion to be detected in lower frequency areas.

Sleep times (currently light sleep due to need to preserve last image ... this may be reduced in future to fit in RTC RAM, so that deep sleep may be used
instead), vary depending on whether or not movement has been detected. Captured images are stored on SD card and uploaded onto a server every 'n' images, 
after which they are deleted from the SD card. WIFI is only enabled during image upload in an effort to reduce power useage. 

The SD card write is somewhat slow. A future improvement will be to farm off the motion detection to the 2nd core while the first is managing the write, 
thus reducing the time the camera is active.
