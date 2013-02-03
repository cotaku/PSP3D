#ifndef __DEF_H_
#define __DEF_H_

#include "math.h"
#include "vector.h"
#include "debug.h"

#define EPSILON 1e-8

#define BUF_WIDTH 512
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 272
#define AspectRatio ((float)SCREEN_WIDTH / (float)SCREEN_HEIGHT)
#define FRAMEBUFFER_SIZE (BUF_WIDTH * SCREEN_HEIGHT * sizeof(int))

extern void* g_p_displayList; // holds the data of our display list, for example the render states of our program is saved on this list.
extern void* g_p_fbp0;    // the start of the framebuffer

#define RGB(r, g, b) ((((b)&0xff)<<16)|(((g)&0xff)<<8)|((r)&0xff))
#define RGBA(r, g, b, a) ((((a)&0xff)<<24)|(((b)&0xff)<<16)|(((g)&0xff)<<8)|((r)&0xff))
#define ColorRed Color(255, 0, 0)
#define ColorGreen Color(0, 255, 0)
#define ColorBlue Color(0, 0, 255)
#define ColorYellow Color(255, 255, 0)
#define ColorOrange Color(255, 85, 0)
#define ColorPurple Color(184, 72, 255)
#define ColorGray Color(127, 127, 127)
#define ColorBlack Color(0, 0, 0)
#define ColorWhite Color(255, 255, 255)

typedef struct _Color
{
    _Color() { }
    inline _Color(char r, char g, char b)
    {
        R = r;
        G = g;
        B = b;
        A = 0;
    }
    inline _Color(u32 color)
    {
        R = color&0xff;
        G = (color>>8)&0xff;
        B = (color>>16)&0xff;
        A = (color>>24)&0xff;
    }
    inline _Color& operator=(const _Color& color)
    {		
        R = color.R;
        G = color.G;
        B = color.B;
        A = color.A;
        return *this;		
    }
    void Print() const
    {
        DebugQuickPrint("R=%d,G=%d,B=%d\n",R,G,B);
    }
    u8 R, G, B, A;
} Color;

typedef enum 
{
    TRANSFORM_TRANSLATE,
    TRANSFORM_ROTATE,
    TRANSFORM_SCALE
} Transformation;

typedef enum
{
    POINT_MODE,
    FRAME_MODE,
    NORMAL_MODE
} RenderMode;

typedef enum
{ 
    LEFT,
    RIGHT,
    BOTTOM, 
    TOP,
    HITHER,
    YON
}CLIP;

#endif
