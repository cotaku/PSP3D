#ifndef __PIPE_H_
#define __PIPE_H_

#include <string.h>
#include <pspdisplay.h>
#include "def.h"
#include "psp3dbasic.h"

void DrawLine(Color color, int x0, int y0, int x1, int y1);
void FillRect(Color color, int x0, int y0, int width, int height);

class VideoBuffer
{
public:
    static VideoBuffer* GetInstance()  
    {  
        if(_pInstance == NULL)  
            _pInstance = new VideoBuffer();  
        return _pInstance;  
    }

    inline void ClearDepth()
    {
        memcpy(_depthBuf, _depthInitBuf, SCREEN_HEIGHT*SCREEN_WIDTH*sizeof(float));
    }

    inline void ClearScreen()
    {
        memset(getVramDisplayBuffer(), 0, sizeof(int)*BUF_WIDTH*SCREEN_HEIGHT);
    }

    void Render(vector<Point3>& points, vector<unsigned int>& indices, RenderMode mode);

private:
    VideoBuffer()
    {
        _vram[0] = (Color*)(0x40000000 | 0x04000000);
        _vram[1] = (Color*)((0x40000000 | 0x04000000) + FRAMEBUFFER_SIZE);
        _dispBufferNumber = 0;
        for (int y = 0; y < SCREEN_HEIGHT; y++)
        {
            for (int x = 0; x < SCREEN_WIDTH; x++)
                _depthInitBuf[y*SCREEN_WIDTH + x] = 1.0;
        }
        _fps = 0;
    }

    ~VideoBuffer()
    {
        if(_pInstance != NULL)  
            delete _pInstance;
    }

    inline Color* getVramDisplayBuffer()
    {
        return _vram[_dispBufferNumber];
    }

    inline void flipBuffer()
    {
        _dispBufferNumber = 1 - _dispBufferNumber;
    }

    inline void setDepth(int x, int y, float depth)
    {
        _depthBuf[y*SCREEN_WIDTH + x] = depth;
    }
    inline float getDepth(int x, int y) const
    {
        return _depthBuf[y*SCREEN_WIDTH + x];
    }

    inline void drawPixel(Color* pVram, int x, int y, Color color)
    {
        *(pVram + (SCREEN_HEIGHT - 1 - y)*BUF_WIDTH + x) = color;
    }

    inline void drawLine(Color* pVram, int x0, int y0, int x1, int y1, Color color)
    {
        int x, y, dx, dy, dx2, dy2, xstep, ystep, error, index;
        x = x0;
        y = y0;
        dx = x1 - x0;
        dy = y1 - y0;

        if (dx >= 0)
        {
            xstep = 1;
        }
        else
        {
            xstep = -1;
            dx = -dx;
        }

        if (dy >= 0)
        {
            ystep = 1;
        }
        else
        {
            ystep = -1;
            dy = -dy;
        }

        dx2 = dx << 1;
        dy2 = dy << 1;

        if (dx > dy)
        {
            error = dy2 - dx;
            for (index = 0; index <= dx; index++)
            {
                drawPixel(pVram, x, y, color);
                if (error >= 0)
                {
                    error -= dx2;
                    y += ystep;
                }
                error += dy2;
                x += xstep;
            }
        }
        else
        {
            error = dx2 - dy;
            for (index = 0; index <= dy; index++)
            {
                drawPixel(pVram, x, y, color);
                if (error >= 0)
                {
                    error -= dy2;
                    x += xstep;
                }
                error += dx2;
                y += ystep;
            }
        }
    }

    void drawTriangleFlatBottom(Color* pVram, int xtop, int ytop, Color ctop, int x1, Color c1, int x2, Color c2, int ybtm, float A, float B, float C, float D);

    void drawTriangleFlatTop(Color* pVram, int xbtm, int ybtm, Color cbtm, int x1, Color c1, int x2, Color c2, int ytop, float A, float B, float C, float D);

    void rasterize(Color* pVram, const Point3 &p1, const Point3 &p2, const Point3 &p3, RenderMode mode);

    void fps(int);

    static VideoBuffer* _pInstance;
    Color* _vram[2];
    float _depthBuf[SCREEN_HEIGHT * SCREEN_WIDTH];
    float _depthInitBuf[SCREEN_HEIGHT * SCREEN_WIDTH];
    int _dispBufferNumber;

    int _fps;
    char _fpsDisplay[128];
    static u32 _tickResolution;
    u64 _fpsTickNow;
    u64 _fpsTickLast;
};

inline void ApplyTransformation(const Matrix4& m_tran, Point3& point)
{
    point = m_tran * point;
}

void InitiateGraphicEngine();

void GetViewTransformation(const Point3 &eye, const Point3 &interest, const Vector3 &up, Matrix4& mat);
void GetPerspectiveTransformation(float xfov, float aspectRatio, float znear, float zfar, Matrix4& mat);

void Clip(CLIP clipSide, vector<Point3> *points, vector<unsigned int> *indices);
void Cull(vector<Point3> *points, vector<unsigned int> *indices);

#endif
