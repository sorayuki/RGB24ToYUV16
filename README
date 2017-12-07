# RGB24ToYUV16 Avisynth plugin manual

A simple plugin to convert RGB24 frame to high depth (10bit or 16bit) YUV.

## Function reference

### RGB24ToYUV

```
RGB24ToYUV(clip sourceClip, string outputFormat = "TV BT709 10BIT STACKED I420")
```

#### Arguments

##### sourceClip
Input clip to do RGB24 to YUV conversion.

If the clip has a width or height not mod4, convert to I420 will output broken frame.

##### outputFormat
One or more words to identify what format to output. 

Available words are:

Range:
- TV
- PC

Color matrix:
- BT601
- BT709
- BT2020
- YCgCo

Bit depth:
- 16BIT
- 10BIT

Color space:
- I444
- I420

Hacked mode:
- INTERLEAVED
- STACKED
