# The interfaces Package

The interfaces package is in charge of playing sounds on demand that are stored in your computer using the `.wav` format. 

Audio files are expected to exist in the [audio](../../../../media/audio/) folder. To play a sound you can publish a string to the `/device/speaker/command` topic containing the name of the file you want to play: 

```
ros2 topic pub -1 /device/speaker/command std_msgs/msg/String '{data: parking.wav}'
```
You will get a warning if you try to play an audio that does not exist.
