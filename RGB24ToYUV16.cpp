#define AVS_LINKAGE_DLLIMPORT

#include <Windows.h>
#include "avisynth.h"

#include <iostream>
#include <cstdint>
#include <algorithm>
#include <thread>
#include <future>
#include <chrono>

#include <emmintrin.h>

#define PLUGIN_NAME "RGB24ToYUV16"

//#define PERFORMANCE_TEST

#define MULTITHREADS 1

//.8 fixed

#define STACKED true
#define INTERLEAVED false

namespace {

    inline void VecMulMat(int32_t* vec, int32_t(*mat)[4])
    {
        int32_t result[3] = { mat[0][3] << 8, mat[1][3] << 8, mat[2][3] << 8 };

        for (int j = 0; j < 3; ++j)
            for (int i = 0; i < 3; ++i)
                result[j] += vec[i] * mat[j][i];

        for (int i = 0; i < 3; ++i)
            vec[i] = result[i] >> 8;
    }

    inline void MatMulMat(int32_t(*mat1)[4], const std::initializer_list<int32_t(*)[4]>& mats)
    {
        for (int32_t(*mat2)[4] : mats)
        {
            int32_t result[4][4];
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x)
                    result[y][x] = mat1[y][0] * mat2[0][x] + mat1[y][1] * mat2[1][x] + mat1[y][2] * mat2[2][x] + mat1[y][3] * mat2[3][x];
            for (int y = 0; y < 4; ++y)
                for (int x = 0; x < 4; ++x)
                    mat1[y][x] = result[y][x] >> 8;
        }
    }

    int32_t Imat[4][4] = 
    {
        0x100, 0, 0, 0,
        0, 0x100, 0, 0,
        0, 0, 0x100, 0,
        0, 0, 0, 0x100
    };

    int32_t ToPCRange[4][4] =
    {
        0x100, 0, 0, 0,
        0, 0x100, 0, 0x8000,
        0, 0, 0x100, 0x8000,
        0, 0, 0, 0x100
    };

    int32_t ToTVRange[4][4] =
    {
        0xDC, 0, 0, 0x1000,
        0, 0xE1, 0, 0x8000,
        0, 0, 0xE1, 0x8000,
        0, 0, 0, 0x100
    };

    int32_t FromRGB24[4][4] =
    {
        0x100, 0, 0, 0,
        0, 0x100, 0, 0,
        0, 0, 0x100, 0,
        0, 0, 0, 0x100
    };

    int32_t ToYCgCo[4][4] =
    {
        0x40, 0x80, 0x40, 0,
        -0x40, 0x80, -0x40, 0,
        0x80, 0, -0x80, 0,
        0, 0, 0, 0x100
    };

#define ToFixPt(x) static_cast<int32_t>((x) * 256)

#define DeclToYUVMatrix(Name, Kr, Kg, Kb) \
    int32_t Name[4][4] = \
    {\
        ToFixPt(Kr), ToFixPt(Kg), ToFixPt(Kb), 0, \
        ToFixPt(-0.5 * Kr / (1 - Kb)), ToFixPt(-0.5 * Kg / (1 - Kb)), ToFixPt(0.5), 0, \
        ToFixPt(0.5), ToFixPt(-0.5 * Kg / (1 - Kr)), ToFixPt(-0.5 * Kb / (1 - Kr)), 0, \
        0, 0, 0, 0x100 \
    };

    DeclToYUVMatrix(ToBT709, 0.2125, 0.7154, 0.0721);

    DeclToYUVMatrix(ToBT601, 0.299, 0.587, 0.114);

    DeclToYUVMatrix(ToBT2020, 0.2627, 0.678, 0.0593);

    /*
    int32_t ToBT709[4][4] =
    {
        0x36, 0xb7, 0x12, 0,
        -0x1d, -0x62, 0x80, 0,
        0x80, -0x74, -0xb, 0,
        0, 0, 0, 0x100
    };
    */

    

    struct RGB24_OP
    {
        enum { AVSCS = VideoInfo::CS_BGR24 };

        uint8_t* pData;
        int stride;
        int height;

        uint8_t* pCurRow;
        uint8_t* pCurSample;

        RGB24_OP(PVideoFrame& pv, bool ro)
        {
            if (ro)
                pData = const_cast<uint8_t*>(pv->GetReadPtr());
            else
                pData = pv->GetWritePtr();

            stride = pv->GetPitch();
            height = pv->GetHeight();

            pData += stride * height;

            Reset();
        }

        void Reset()
        {
            pCurRow = pData + height * stride;
            pCurRow -= stride;
            pCurSample = pCurRow;
        }

        void JumpToRow(int row)
        {
            pCurRow = pData - stride - row * stride;
            pCurSample = pCurRow;
        }

        void NextRow()
        {
            pCurRow -= stride;
            pCurSample = pCurRow;
        }

        void NextReadSample()
        {
            pCurSample += 3;
        }

        void Read(int32_t data[3])
        {
            data[2] = ((const uint8_t*)pCurSample)[0] * 0x101;
            data[1] = ((const uint8_t*)pCurSample)[1] * 0x101;
            data[0] = ((const uint8_t*)pCurSample)[2] * 0x101;
        }
    };

    struct YV12_OP
    {
        enum { AVSCS = VideoInfo::CS_YV12 };

        uint8_t *pY, *pU, *pV;
        int pitchY, pitchU, pitchV;

        uint8_t *pYr, *pUr, *pVr;
        uint8_t *pYs, *pUs, *pVs;

        int x, y;

        int prevU, prevV;

        int height;
        int halfheight;

        YV12_OP(PVideoFrame& pv, bool ro)
        {
            height = pv->GetHeight();
            halfheight = height / 2;

            if (ro)
            {
                pY = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_Y));
                pU = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_U));
                pV = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_V));
            }
            else
            {
                pY = pv->GetWritePtr(PLANAR_Y);
                pU = pv->GetWritePtr(PLANAR_U);
                pV = pv->GetWritePtr(PLANAR_V);
            }
            pitchY = pv->GetPitch(PLANAR_Y);
            pitchU = pv->GetPitch(PLANAR_U);
            pitchV = pv->GetPitch(PLANAR_V);

            Reset();
        }

        void Reset()
        {
            pYr = pYs = pY;
            pUr = pUs = pU;
            pVr = pVs = pV;
            x = 0;
            y = 0;
        }

        void JumpToRow(int row)
        {
            pYs = pYr = pY + pitchY * row;
            pUs = pUr = pU + pitchU * (row / 2);
            pVs = pVr = pV + pitchV * (row / 2);
            y = row;
            x = 0;
        }

        void NextRow()
        {
            ++y;
            x = 0;

            pYr += pitchY;
            pYs = pYr;
            if ((y & 1) == 0)
            {
                pUr += pitchU;
                pVr += pitchV;
            }
            pUs = pUr;
            pVs = pVr;
        }

        void NextReadSample()
        {
            ++x;
            ++pYs;
            if ((x & 1) == 0)
            {
                ++pUs;
                ++pVs;
            }
        }

        void Read(int32_t data[3])
        {
            data[0] = ((const uint8_t*)pYs)[0] << 8 | ((const uint8_t*)pYs)[0];
            data[1] = ((const uint8_t*)pUs)[0] << 8 | ((const uint8_t*)pUs)[0];
            data[2] = ((const uint8_t*)pVs)[0] << 8 | ((const uint8_t*)pVs)[0];
        }

        template<bool isStack> void NextWriteSample();
        template<bool isStack> void Write(int32_t data[3]);

        template<> void NextWriteSample<INTERLEAVED>()
        {
            ++x;
            pYs += 2;
            if ((x & 1) == 0)
            {
                pUs += 2;
                pVs += 2;
            }
        }

        template<> void NextWriteSample<STACKED>()
        {
            NextReadSample();
        }

        template<> void Write<INTERLEAVED>(int32_t data[3])
        {
            pYs[1] = data[0] >> 8;
            pYs[0] = data[0] & 0xff;

            prevU = data[1];
            prevV = data[2];

            if ((x & 1) == 0 && (y & 1) == 0)
            {
                pUs[1] = 0;
                pUs[0] = 0;
                pVs[1] = 0;
                pVs[0] = 0;
            }

            int16_t u = (pUs[0] << 8 | pUs[1]) + (data[1] >> 2);
            int16_t v = (pVs[0] << 8 | pVs[1]) + (data[2] >> 2);

            pUs[1] = u >> 8;
            pUs[0] = u & 0xff;
            pVs[1] = v >> 8;
            pVs[0] = v & 0xff;
        }

        template<> void Write<STACKED>(int32_t data[3])
        {
            pYs[0] = data[0] >> 8;
            pYs[(height >> 1) * pitchY] = data[0] & 0xff;
            if ((x & 1) == 0 && (y & 1) == 0)
            {
                pUs[0] = 0;
                pUs[(halfheight >> 1) * pitchU] = 0;
                pVs[0] = 0;
                pVs[(halfheight >> 1) * pitchV] = 0;
            }

            int16_t u = (pUs[0] << 8 | pUs[(halfheight >> 1) * pitchU]) + (data[1] >> 2);
            int16_t v = (pVs[0] << 8 | pVs[(halfheight >> 1) * pitchV]) + (data[2] >> 2);

            pUs[0] = u >> 8;
            pUs[(halfheight >> 1) * pitchU] = u & 0xff;
            pVs[0] = v >> 8;
            pVs[(halfheight >> 1) * pitchV] = v & 0xff;
        }
    };

    struct YV24_OP
    {
        enum { AVSCS = VideoInfo::CS_YV24 };

        uint8_t *pY, *pU, *pV;
        int pitchY, pitchU, pitchV;

        uint8_t *pYr, *pUr, *pVr;
        uint8_t *pYs, *pUs, *pVs;

        int height;
        int halfheight;

        YV24_OP(PVideoFrame& pv, bool ro)
        {
            height = pv->GetHeight();
            halfheight = height / 2;

            if (ro)
            {
                pY = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_Y));
                pU = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_U));
                pV = const_cast<uint8_t*>(pv->GetReadPtr(PLANAR_V));
            }
            else
            {
                pY = pv->GetWritePtr(PLANAR_Y);
                pU = pv->GetWritePtr(PLANAR_U);
                pV = pv->GetWritePtr(PLANAR_V);
            }
            pitchY = pv->GetPitch(PLANAR_Y);
            pitchU = pv->GetPitch(PLANAR_U);
            pitchV = pv->GetPitch(PLANAR_V);

            Reset();
        }

        void Reset()
        {
            pYr = pYs = pY;
            pUr = pUs = pU;
            pVr = pVs = pV;
        }

        void JumpToRow(int row)
        {
            pYs = pYr = pY + pitchY * row;
            pUs = pUr = pU + pitchU * row;
            pVs = pVr = pV + pitchV * row;
        }

        void NextRow()
        {
            pYr += pitchY;
            pYs = pYr;

            pUr += pitchU;
            pVr += pitchV;
            pUs = pUr;
            pVs = pVr;
        }

        void NextReadSample()
        {
            ++pYs;
            ++pUs;
            ++pVs;
        }

        void Read(int32_t data[3])
        {
            data[0] = pYs[0] * 0x101;
            data[1] = pUs[0] * 0x101;
            data[2] = pVs[0] * 0x101;
        }

        template<bool isStack> void NextWriteSample();
        template<bool isStack> void Write(int32_t data[3]);

        template<> void NextWriteSample<INTERLEAVED>()
        {
            pYs += 2;
            pUs += 2;
            pVs += 2;
        }

        template<> void NextWriteSample<STACKED>()
        {
            NextReadSample();
        }

        template<> void Write<INTERLEAVED>(int32_t data[3])
        {
            pYs[1] = data[0] >> 8;
            pYs[0] = data[0] & 0xff;

            pUs[1] = data[1] >> 8;
            pUs[0] = data[1] & 0xff;
            pVs[1] = data[2] >> 8;
            pVs[0] = data[2] & 0xff;
        }

        template<> void Write<STACKED>(int32_t data[3])
        {
            pYs[0] = data[0] >> 8;
            pYs[halfheight * pitchY] = data[0] & 0xff;
            pUs[0] = data[1] >> 8;
            pUs[halfheight * pitchU] = data[1] & 0xff;
            pVs[0] = data[2] >> 8;
            pVs[halfheight * pitchV] = data[2] & 0xff;
        }
    };
};

template<typename InOP, typename OutOP, bool isStack, int offset>
class Converter : public GenericVideoFilter
{
    VideoInfo inVi;
    int32_t mat_[4][4];

public:
    Converter(PClip clip, int32_t (*mat)[4], IScriptEnvironment* env) : GenericVideoFilter(clip)
    {
        inVi = GetVideoInfo();

        if (inVi.pixel_type != InOP::AVSCS)
            env->ThrowError(PLUGIN_NAME ": incorrect input colorspace.");

        if (isStack)
            vi.height *= 2;
        else
            vi.width *= 2;

        vi.pixel_type = OutOP::AVSCS;

        memcpy(mat_, mat, sizeof(mat_));
    }

    PVideoFrame __stdcall GetFrame(int nFrame, IScriptEnvironment* env) override
    {
        PVideoFrame inFrame = GenericVideoFilter::GetFrame(nFrame, env);
        PVideoFrame outFrame = env->NewVideoFrame(vi);

        InOP pinop(inFrame, true);
        OutOP poutop(outFrame, false);

        auto task = [this, pinop, poutop](int sh, int eh){
            InOP inop = pinop;
            OutOP outop = poutop;

            for (int line = sh; line < eh; ++line)
            {
                int32_t samples[3];
                inop.JumpToRow(line);
                outop.JumpToRow(line);

                for (int x = 0; x < inVi.width; ++x)
                {
                    inop.Read(samples);
                    VecMulMat(samples, mat_);
                    if (offset > 0)
                    {
                        for (int i = 0; i < 3; ++i)
                            samples[i] >>= offset;
                    }
                    outop.template Write<isStack>(samples);

                    inop.NextReadSample();
                    outop.template NextWriteSample<isStack>();
                }
            }
        };

#if MULTITHREADS
        int cpuCount = std::thread::hardware_concurrency();
        cpuCount = min(cpuCount, 8);

        std::future<void> futures[8];

        int sh = 0, eh;
        int hstep = (inVi.height / cpuCount) & ~0x3; //mod 4
        for (int i = 0; i < cpuCount; ++i)
        {
            eh = sh + hstep;
            if (i + 1 == cpuCount)
                eh = inVi.height;

            futures[i] = std::async(task, sh, eh);

            sh = eh;
        }

        for (int i = 0; i < cpuCount; ++i)
            futures[i].get();

#elif 1
        task(0, vi.height);
#endif

#ifdef PERFORMANCE_TEST
        std::chrono::system_clock::time_point onEnd = std::chrono::system_clock::now();
        std::cerr << "cost: " << (onEnd - onBegin).count() << std::endl;
#endif

        return outFrame;
    }
};



#if 0

template<int offset>
class RGB24ToYCgCo : public GenericVideoFilter
{
    VideoInfo inVi;
public:
    RGB24ToYCgCo(PClip clip, IScriptEnvironment* env) : GenericVideoFilter(clip)
    {
        if (GetVideoInfo().pixel_type != VideoInfo::CS_BGR24)
            env->ThrowError("RGB24ToYCgCo: BGR24 input only.");

        inVi = GetVideoInfo();

        vi.width *= 2;
        vi.pixel_type = VideoInfo::CS_YV24;
    }

    PVideoFrame __stdcall GetFrame(int nFrame, IScriptEnvironment* env) override
    {
        PVideoFrame inFrame = GenericVideoFilter::GetFrame(nFrame, env);
        PVideoFrame outFrame = env->NewVideoFrame(vi);
     
#ifdef PERFORMANCE_TEST
        std::chrono::system_clock::time_point onBegin = std::chrono::system_clock::now();
#endif

        BYTE* yptr = outFrame->GetWritePtr(PLANAR_Y);
        int ypitch = outFrame->GetPitch(PLANAR_Y);
        BYTE* uptr = outFrame->GetWritePtr(PLANAR_U);
        int upitch = outFrame->GetPitch(PLANAR_U);
        BYTE* vptr = outFrame->GetWritePtr(PLANAR_V);
        int vpitch = outFrame->GetPitch(PLANAR_V);

        const BYTE* sptr = inFrame->GetReadPtr();
        int spitch = inFrame->GetPitch();

        auto task = [this, yptr, ypitch, uptr, upitch, vptr, vpitch, sptr, spitch](int sh, int eh){
            for (int line = sh; line < eh; ++line)
            {
                std::uint16_t* ylptr = (std::uint16_t*)(yptr + ypitch * line);
                std::uint16_t* ulptr = (std::uint16_t*)(uptr + upitch * line);
                std::uint16_t* vlptr = (std::uint16_t*)(vptr + vpitch * line);
                const BYTE* slptr = sptr + spitch * (inVi.height - 1 - line);

                for (int x = 0; x < inVi.width; ++x)
                {
                    const BYTE* s = &slptr[x * 3];
                    uint16_t r = (s[2] << 8) | s[2];
                    uint16_t g = (s[1] << 8) | s[1];
                    uint16_t b = (s[0] << 8) | s[0];

                    ylptr[x] = ((r >> 2) + (g >> 1) + (b >> 2)) >> offset;
                    ulptr[x] = ((-(r >> 2) + (g >> 1) - (b >> 2)) + 0x8000) >> offset;
                    vlptr[x] = (((r >> 1) - (b >> 1)) + 0x8000) >> offset;

                    std::swap(*(char*)ylptr, *((char*)ylptr + 1));
                    std::swap(*(char*)ulptr, *((char*)ulptr + 1));
                    std::swap(*(char*)vlptr, *((char*)vlptr + 1));
                }
            }
        };

#if 1
        int cpuCount = std::thread::hardware_concurrency();
        cpuCount = min(cpuCount, 8);

        std::future<void> futures[8];

        for (int i = 0; i < cpuCount; ++i)
        { 
            int sh, eh;
            sh = inVi.height / cpuCount * i;
            eh = sh + inVi.height / cpuCount;
            if (i + 1 == cpuCount)
                eh = inVi.height;

            futures[i] = std::async(task, sh, eh);
        }

        for (int i = 0; i < cpuCount; ++i)
            futures[i].get();

#elif 1
        task(0, vi.height);
#endif

#ifdef PERFORMANCE_TEST
        std::chrono::system_clock::time_point onEnd = std::chrono::system_clock::now();
        std::cerr << "cost: " << (onEnd - onBegin).count() << std::endl;
#endif

        return outFrame;
    }
};

class RGB24ToYCgCo16S : public GenericVideoFilter
{
    VideoInfo inVi;
public:
    RGB24ToYCgCo16S(PClip clip, IScriptEnvironment* env) : GenericVideoFilter(clip)
    {
        if (GetVideoInfo().pixel_type != VideoInfo::CS_BGR24)
            env->ThrowError("RGB24ToYCgCo16S: BGR24 input only.");

        inVi = GetVideoInfo();

        vi.height *= 2;
        vi.pixel_type = VideoInfo::CS_YV24;
    }

    PVideoFrame __stdcall GetFrame(int nFrame, IScriptEnvironment* env) override
    {
        PVideoFrame inFrame = GenericVideoFilter::GetFrame(nFrame, env);
        PVideoFrame outFrame = env->NewVideoFrame(vi);

#ifdef PERFORMANCE_TEST
        std::chrono::system_clock::time_point onBegin = std::chrono::system_clock::now();
#endif

        BYTE* yptr = outFrame->GetWritePtr(PLANAR_Y);
        int ypitch = outFrame->GetPitch(PLANAR_Y);
        BYTE* uptr = outFrame->GetWritePtr(PLANAR_U);
        int upitch = outFrame->GetPitch(PLANAR_U);
        BYTE* vptr = outFrame->GetWritePtr(PLANAR_V);
        int vpitch = outFrame->GetPitch(PLANAR_V);

        const BYTE* sptr = inFrame->GetReadPtr();
        int spitch = inFrame->GetPitch();

        auto task = [this, yptr, ypitch, uptr, upitch, vptr, vpitch, sptr, spitch](int sh, int eh){
            for (int line = sh; line < eh; ++line)
            {
                std::uint8_t* ymptr = (std::uint8_t*)(yptr + ypitch * line);
                std::uint8_t* umptr = (std::uint8_t*)(uptr + upitch * line);
                std::uint8_t* vmptr = (std::uint8_t*)(vptr + vpitch * line);
                std::uint8_t* ylptr = ymptr + inVi.height * ypitch;
                std::uint8_t* ulptr = umptr + inVi.height * upitch;
                std::uint8_t* vlptr = vmptr + inVi.height * vpitch;

                const BYTE* slptr = sptr + spitch * (inVi.height - 1 - line);

                for (int x = 0; x < inVi.width; ++x)
                {
                    const BYTE* s = &slptr[x * 3];
                    uint16_t r = (s[2] << 8) | s[2];
                    uint16_t g = (s[1] << 8) | s[1];
                    uint16_t b = (s[0] << 8) | s[0];

                    uint16_t y = ((r >> 2) + (g >> 1) + (b >> 2));
                    uint16_t u = ((-(r >> 2) + (g >> 1) - (b >> 2)) + 0x8000);
                    uint16_t v = (((r >> 1) - (b >> 1)) + 0x8000);

                    ymptr[x] = y >> 8;
                    ylptr[x] = y & 0xff;
                    umptr[x] = u >> 8;
                    ulptr[x] = u & 0xff;
                    vmptr[x] = v >> 8;
                    vlptr[x] = v & 0xff;
                }
            }
        };

#if 1
        int cpuCount = std::thread::hardware_concurrency();
        cpuCount = min(cpuCount, 8);

        std::future<void> futures[8];

        for (int i = 0; i < cpuCount; ++i)
        {
            int sh, eh;
            sh = inVi.height / cpuCount * i;
            eh = sh + inVi.height / cpuCount;
            if (i + 1 == cpuCount)
                eh = inVi.height;

            futures[i] = std::async(task, sh, eh);
        }

        for (int i = 0; i < cpuCount; ++i)
            futures[i].get();

#elif 1
        task(0, vi.height);
#endif

#ifdef PERFORMANCE_TEST
        std::chrono::system_clock::time_point onEnd = std::chrono::system_clock::now();
        std::cerr << "cost: " << (onEnd - onBegin).count() << std::endl;
#endif

        return outFrame;
    }
};

#endif

template<typename SrcOP, typename DstOP, int bitdepth, bool isStack, int32_t(*mat1)[4], int32_t(*mat2)[4], int32_t(*mat3)[4] = Imat, int32_t(*mat4)[4] = Imat>
AVSValue __cdecl CreateRGB24ToYUV(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    int32_t mat[4][4];
    memcpy(mat, Imat, sizeof(mat));
    MatMulMat(mat, { mat1, mat2, mat3, mat4 });
    return new Converter<SrcOP, DstOP, isStack, 16 - bitdepth>(args[0].AsClip(), mat, env);
}

struct ConvertParam
{
    enum
    {
        CS_YUV420,
        CS_YUV444,

        MAT_BT601,
        MAT_BT709,
        MAT_BT2020,
        MAT_YCgCo,

        RANGE_PC,
        RANGE_TV,

        HACK_STACKED,
        HACK_INTERLEAVED,

        BIT_10,
        BIT_16
    };

    int outMat;
    int outRange;
    int outDepth;
    int outMode;
    int outCS;

    ConvertParam()
    {
        outMat = MAT_BT709;
        outRange = RANGE_TV;
        outDepth = BIT_10;
        outMode = HACK_STACKED;
        outCS = CS_YUV420;
    }
};

template<typename DstOP, int bitdepth, bool isStack, int32_t(*mat1)[4]>
AVSValue __cdecl CreateRGB24ToYUV4(ConvertParam* cp, AVSValue args, void* user_data, IScriptEnvironment* env)
{
    if (cp->outMat == ConvertParam::MAT_BT601)
        return CreateRGB24ToYUV<RGB24_OP, DstOP, bitdepth, isStack, mat1, ToBT601, FromRGB24>(args, user_data, env);
    else if (cp->outMat == ConvertParam::MAT_BT709)
        return CreateRGB24ToYUV<RGB24_OP, DstOP, bitdepth, isStack, mat1, ToBT709, FromRGB24>(args, user_data, env);
    else if (cp->outMat == ConvertParam::MAT_BT2020)
        return CreateRGB24ToYUV<RGB24_OP, DstOP, bitdepth, isStack, mat1, ToBT2020, FromRGB24>(args, user_data, env);
    else if (cp->outMat == ConvertParam::MAT_YCgCo)
        return CreateRGB24ToYUV<RGB24_OP, DstOP, bitdepth, isStack, mat1, ToYCgCo, FromRGB24>(args, user_data, env);
    else
        env->ThrowError(PLUGIN_NAME ": Internal Error.");
}

template<int bitdepth, bool isStack, int32_t(*mat1)[4]>
AVSValue __cdecl CreateRGB24ToYUV3(ConvertParam* cp, AVSValue args, void* user_data, IScriptEnvironment* env)
{
    if (cp->outCS == ConvertParam::CS_YUV420)
        return CreateRGB24ToYUV4<YV12_OP, bitdepth, isStack, mat1>(cp, args, user_data, env);
    else if (cp->outCS == ConvertParam::CS_YUV444)
        return CreateRGB24ToYUV4<YV24_OP, bitdepth, isStack, mat1>(cp, args, user_data, env);
    else
        env->ThrowError(PLUGIN_NAME ": Internal Error.");
}

template<int bitdepth, bool isStack>
AVSValue __cdecl CreateRGB24ToYUV2(ConvertParam* cp, AVSValue args, void* user_data, IScriptEnvironment* env)
{
    if (cp->outRange == ConvertParam::RANGE_PC)
        return CreateRGB24ToYUV3<bitdepth, isStack, ToPCRange>(cp, args, user_data, env);
    else if (cp->outRange == ConvertParam::RANGE_TV)
        return CreateRGB24ToYUV3<bitdepth, isStack, ToTVRange>(cp, args, user_data, env);
    else
        env->ThrowError(PLUGIN_NAME ": Internal Error.");
}

template<int bitdepth>
AVSValue __cdecl CreateRGB24ToYUV1(ConvertParam* cp, AVSValue args, void* user_data, IScriptEnvironment* env)
{
    if (cp->outMode == ConvertParam::HACK_INTERLEAVED)
        return CreateRGB24ToYUV2<bitdepth, INTERLEAVED>(cp, args, user_data, env);
    else if (cp->outMode == ConvertParam::HACK_STACKED)
        return CreateRGB24ToYUV2<bitdepth, STACKED>(cp, args, user_data, env);
    else
        env->ThrowError(PLUGIN_NAME ": Internal Error.");
}

AVSValue __cdecl CreateRGB24ToYUV0(ConvertParam* cp, AVSValue args, void* user_data, IScriptEnvironment* env)
{
    if (cp->outDepth == ConvertParam::BIT_10)
        return CreateRGB24ToYUV1<10>(cp, args, user_data, env);
    else if (cp->outDepth == ConvertParam::BIT_16)
        return CreateRGB24ToYUV1<16>(cp, args, user_data, env);
    else
        env->ThrowError(PLUGIN_NAME ": Internal Error.");
}

AVSValue __cdecl CreateRGB24ToYUV(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    char* paramStr = _strdup(args[1].AsString());

    ConvertParam cp;

    char* p = paramStr;
    char* pend = paramStr + strlen(paramStr);
    
    while (p < pend)
    {
        p = strtok(p, " ");

        if (strcmp(p, "TV") == 0)
            cp.outRange = ConvertParam::RANGE_TV;
        else if (strcmp(p, "PC") == 0)
            cp.outRange = ConvertParam::RANGE_PC;
        else if (strcmp(p, "BT601") == 0)
            cp.outMat = ConvertParam::MAT_BT601;
        else if (strcmp(p, "BT709") == 0)
            cp.outMat = ConvertParam::MAT_BT709;
        else if (strcmp(p, "BT2020") == 0)
            cp.outMat = ConvertParam::MAT_BT2020;
        else if (strcmp(p, "YCgCo") == 0)
            cp.outMat = ConvertParam::MAT_YCgCo;
        else if (strcmp(p, "16BIT") == 0)
            cp.outDepth = ConvertParam::BIT_16;
        else if (strcmp(p, "10BIT") == 0)
            cp.outDepth = ConvertParam::BIT_10;
        else if (strcmp(p, "I444") == 0)
            cp.outCS = ConvertParam::CS_YUV444;
        else if (strcmp(p, "I420") == 0)
            cp.outCS = ConvertParam::CS_YUV420;
        else if (strcmp(p, "INTERLEAVED") == 0)
            cp.outMode = ConvertParam::HACK_INTERLEAVED;
        else if (strcmp(p, "STACKED") == 0)
            cp.outMode = ConvertParam::HACK_STACKED;
        else
        {
            char buf[256];
            strcpy_s(buf, p);
            free(paramStr);
            env->ThrowError(PLUGIN_NAME ": unknown param %s", buf);
        }

        p += strlen(p) + 1;
    }

    free(paramStr);
    
    return CreateRGB24ToYUV0(&cp, args, user_data, env);
}

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* const vectors) {
    env->AddFunction("RGB24ToYUV", "cs", &CreateRGB24ToYUV, 0);
    
    return PLUGIN_NAME " Plugin.";
}
