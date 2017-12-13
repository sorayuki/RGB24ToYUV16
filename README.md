# RGB24ToYUV16 Avisynth plugin manual

A simple plugin to convert RGB24 frame to high depth (10bit or 16bit) YUV.

## Function reference

### RGB24ToYUV

```
RGB24ToYUV(clip SourceClip, integer Precision = 10, string HackMode = "STACKED", 
    string ColorSpace = "yv12", string Matrix = "BT709", string Level = "TV")
```

#### Arguments

##### -- SourceClip
Input clip to do RGB24 to YUV conversion.

If the clip has a width or height not mod4, convert to YV12 will output broken frame.

##### -- Precision
- 10: 10-bit depth
- 16: 16-bit depth

##### -- HackMode
- "INTERLEAVED": continuous 2 8-bit depth samples present 1 16-bit or 10-bit sample.
- "STACKED": frame is splited into upper (most significant bits) and lower parts (least significant bits).

##### -- ColorSpace
- "YV12": YUV 420P output
- "YV24": YUV 444P output

##### -- Matrix
output YUV color matrix.
- "BT601"
- "BT709"
- "BT2020"
- "YCgCo"

##### -- Level
value range for output sample
- "TV": 64~940 in 10-bit and 4096~60160 in 16-bit for Y planar; 64~960 in 10-bit and 4096~61440 in 16-bit for UV planar.
- "PC": 0~65535 in 16-bit or 0~1024 in 10bit for all planar.
