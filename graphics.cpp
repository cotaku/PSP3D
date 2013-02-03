#include "pipeline.h"

void* g_p_displayList;
void* g_p_fbp0 = 0;

static int dispBufferNumber = 0;
static Color* g_vram_base = (Color*)(0x40000000 | 0x04000000);

extern VideoBuffer* buf;

void InitiateGraphicEngine()
{
    buf->ClearDepth();
}

static inline Color* GetVramDrawBuffer()
{
    Color* vram = (Color*)g_vram_base;
    if (dispBufferNumber == 0)
        vram += FRAMEBUFFER_SIZE / sizeof(Color);
    return vram;
}

static inline Color* GetVramDisplayBuffer()
{
    Color* vram = (Color*)g_vram_base;
    if (dispBufferNumber == 1) 
        vram += FRAMEBUFFER_SIZE / sizeof(Color);
    return vram;
}

void FillRect(Color color, int x0, int y0, int width, int height)
{
    int x, y;
    Color* data = GetVramDisplayBuffer() + x0 + y0 * BUF_WIDTH;
    for (y = 0; y < height; y++, data += BUF_WIDTH - width) 
    {
        for (x = 0; x < width; x++, data++)
            *data = color;
    }
}
