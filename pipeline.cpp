#include <psptypes.h>
#include <stdio.h>
#include <psprtc.h>
#include "pipeline.h"

VideoBuffer* VideoBuffer::_pInstance = NULL;
u32 VideoBuffer::_tickResolution = sceRtcGetTickResolution();

void VideoBuffer::fps(int vram)
{
    _fps++;
    sceRtcGetCurrentTick(&_fpsTickNow);

    if(((_fpsTickNow - _fpsTickLast)/((float)_tickResolution)) >= 1.0)
    {
        _fpsTickLast = _fpsTickNow;
        sprintf(_fpsDisplay, "FPS: %d", _fps);
        _fps = 0;
    }
    pspDebugScreenSetOffset(vram - (int)_vram[0]);
    pspDebugScreenSetXY(0, 0);
    pspDebugScreenPrintf(_fpsDisplay);
}

void VideoBuffer::drawTriangleFlatBottom(Color* pVram, int xtop, int ytop, Color ctop, int x1, Color c1, int x2, Color c2, int ybtm, float A, float B, float C, float D)
{
    int dx1 = xtop - x1;
    int dy1 = ytop - ybtm;
    int dx2 = xtop - x2;
    int dy2 = ytop - ybtm;
    int xstep1 = 1, xstep2 = 1;
    if (dy1 <= 0 || dy2 <= 0)
        return;

    int r1 = c1.R, g1 = c1.G, b1 = c1.B;
    int r2 = c2.R, g2 = c2.G, b2 = c2.B;
    int dr1 = ctop.R - r1, dg1 = ctop.G - g1, db1 = ctop.B - b1;
    int dr2 = ctop.R - r2, dg2 = ctop.G - g2, db2 = ctop.B - b2;
    int rstep1 = 1, gstep1 = 1, bstep1 = 1;
    int rstep2 = 1, gstep2 = 1, bstep2 = 1;

    if (dx1 < 0)
    {
        xstep1 = -1;
        dx1 = -dx1;
    }
    if (dx2 < 0)
    {
        xstep2 = -1;
        dx2 = -dx2;
    }

    if (dr1 < 0)
    {
        rstep1 = -1;
        dr1 = -dr1;
    }
    if (dg1 < 0)
    {
        gstep1 = -1;
        dg1 = -dg1;
    }
    if (db1 < 0)
    {
        bstep1 = -1;
        db1 = -db1;
    }
    if (dr2 < 0)
    {
        rstep2 = -1;
        dr2 = -dr2;
    }
    if (dg2 < 0)
    {
        gstep2 = -1;
        dg2 = -dg2;
    }
    if (db2 < 0)
    {
        bstep2 = -1;
        db2 = -db2;
    }

    int error1 = 2*dx1 - dy1, error2 = 2*dx2 - dy2;
    int errorr1 = 2*dr1 - dy1, errorg1 = 2*dg1 - dy1, errorb1 = 2*db1 - dy1;
    int errorr2 = 2*dr2 - dy2, errorg2 = 2*dg2 - dy2, errorb2 = 2*db2 - dy2;

    int x1tmp = x1, x2tmp = x2;
    int r1tmp = r1, g1tmp = g1, b1tmp = b1;
    int r2tmp = r2, g2tmp = g2, b2tmp = b2;

    float z0 = (-A*x1 - B*ybtm - D) / C;
    float czx = -A/C, czy = -B/C;

    for (int y = ybtm; y <= ytop; y++)
    {
        while (error1 > 0)
        {
            x1tmp += xstep1;
            error1 -= 2*dy1;
        }
        while (error2 > 0)
        {
            x2tmp += xstep2;
            error2 -= 2*dy2;
        }

        while (errorr1 > 0)
        {
            r1tmp += rstep1;
            errorr1 -= 2*dy1;
        }
        while (errorg1 > 0)
        {
            g1tmp += gstep1;
            errorg1 -= 2*dy1;
        }
        while (errorb1 > 0)
        {
            b1tmp += bstep1;
            errorb1 -= 2*dy1;
        }
        while (errorr2 > 0)
        {
            r2tmp += rstep2;
            errorr2 -= 2*dy2;
        }
        while (errorg2 > 0)
        {
            g2tmp += gstep2;
            errorg2 -= 2*dy2;
        }
        while (errorb2 > 0)
        {
            b2tmp += bstep2;
            errorb2 -= 2*dy2;
        }

        int r = r1tmp, g = g1tmp, b = b1tmp;
        int dr = r2tmp - r1tmp, dg = g2tmp - g1tmp, db = b2tmp - b1tmp;
        int dx = x2tmp - x1tmp;

        int rstep = 1, gstep = 1, bstep = 1;
        if (dr < 0)
        {
            rstep = -1;
            dr = -dr;
        }
        if (dg < 0)
        {
            gstep = -1;
            dg = -dg;
        }
        if (db < 0)
        {
            bstep = -1;
            db = -db;
        }
        int errorr = 2*dr - dx, errorg = 2*dg - dx, errorb = 2*db - dx;

        float z = z0;
        for (int i = x1; i < x1tmp; i++)
            z += czx;

        for (int x = x1tmp; x <= x2tmp; x++)
        {
            if (dx > 0)
            {
                while (errorr > 0)
                {
                    r += rstep;
                    errorr -= 2*dx;
                }
                while (errorg > 0)
                {
                    g += gstep;
                    errorg -= 2*dx;
                }
                while (errorb > 0)
                {
                    b += bstep;
                    errorb -= 2*dx;
                }
            }

            if (z <= getDepth(x, y))
            {
                if (r < 0) r = 0;
                else if (r > 255) r = 255;
                if (g < 0) g = 0;
                else if (g > 255) g = 255;
                if (b < 0) b = 0;
                else if (b > 255) b = 255;
                drawPixel(pVram, x, y, RGBA(r,g,b,0));
                setDepth(x, y, z);
            }

            errorr += 2*dr;
            errorg += 2*dg;
            errorb += 2*db;
            z += czx;
        }

        error1 += 2*dx1;
        error2 += 2*dx2;
        errorr1 += 2*dr1;
        errorg1 += 2*dg1;
        errorb1 += 2*db1;
        errorr2 += 2*dr2;
        errorg2 += 2*dg2;
        errorb2 += 2*db2;
        z0 += czy;
    }
}

void VideoBuffer::drawTriangleFlatTop(Color* pVram, int xbtm, int ybtm, Color cbtm, int x1, Color c1, int x2, Color c2, int ytop, float A, float B, float C, float D)
{
    int dx1 = x1 - xbtm;
    int dy1 = ytop - ybtm;
    int dx2 = x2 - xbtm;
    int dy2 = ytop - ybtm;
    int xstep1 = 1, xstep2 = 1;
    if (dy1 <= 0 || dy2 <= 0)
        return;

    int r1 = c1.R, g1 = c1.G, b1 = c1.B;
    int r2 = c2.R, g2 = c2.G, b2 = c2.B;
    int dr1 = r1 - cbtm.R, dg1 = g1 - cbtm.G, db1 = b1 - cbtm.B;
    int dr2 = r2 - cbtm.R, dg2 = g2 - cbtm.G, db2 = b2 - cbtm.B;
    int rstep1 = 1, gstep1 = 1, bstep1 = 1;
    int rstep2 = 1, gstep2 = 1, bstep2 = 1;

    if (dx1 < 0)
    {
        xstep1 = -1;
        dx1 = -dx1;
    }
    if (dx2 < 0)
    {
        xstep2 = -1;
        dx2 = -dx2;
    }

    if (dr1 < 0)
    {
        rstep1 = -1;
        dr1 = -dr1;
    }
    if (dg1 < 0)
    {
        gstep1 = -1;
        dg1 = -dg1;
    }
    if (db1 < 0)
    {
        bstep1 = -1;
        db1 = -db1;
    }
    if (dr2 < 0)
    {
        rstep2 = -1;
        dr2 = -dr2;
    }
    if (dg2 < 0)
    {
        gstep2 = -1;
        dg2 = -dg2;
    }
    if (db2 < 0)
    {
        bstep2 = -1;
        db2 = -db2;
    }

    int error1 = 2*dx1 - dy1, error2 = 2*dx2 - dy2;
    int errorr1 = 2*dr1 - dy1, errorg1 = 2*dg1 - dy1, errorb1 = 2*db1 - dy1;
    int errorr2 = 2*dr2 - dy2, errorg2 = 2*dg2 - dy2, errorb2 = 2*db2 - dy2;

    int x1tmp = xbtm, x2tmp = xbtm;
    int r1tmp = cbtm.R, g1tmp = cbtm.G, b1tmp = cbtm.B;
    int r2tmp = cbtm.R, g2tmp = cbtm.G, b2tmp = cbtm.B;

    float z0 = (-A*x1 - B*ybtm - D) / C;
    float czx = -A/C, czy = -B/C;
    float z;

    for (int y = ybtm; y < ytop; y++)
    {
        while (error1 > 0)
        {
            x1tmp += xstep1;
            error1 -= 2*dy1;
        }
        while (error2 > 0)
        {
            x2tmp += xstep2;
            error2 -= 2*dy2;
        }

        while (errorr1 > 0)
        {
            r1tmp += rstep1;
            errorr1 -= 2*dy1;
        }
        while (errorg1 > 0)
        {
            g1tmp += gstep1;
            errorg1 -= 2*dy1;
        }
        while (errorb1 > 0)
        {
            b1tmp += bstep1;
            errorb1 -= 2*dy1;
        }
        while (errorr2 > 0)
        {
            r2tmp += rstep2;
            errorr2 -= 2*dy2;
        }
        while (errorg2 > 0)
        {
            g2tmp += gstep2;
            errorg2 -= 2*dy2;
        }
        while (errorb2 > 0)
        {
            b2tmp += bstep2;
            errorb2 -= 2*dy2;
        }

        int r = r1tmp, g = g1tmp, b = b1tmp;
        int dr = r2tmp - r1tmp, dg = g2tmp - g1tmp, db = b2tmp - b1tmp;
        int dx = x2tmp - x1tmp;

        int rstep = 1, gstep = 1, bstep = 1;
        if (dr < 0)
        {
            rstep = -1;
            dr = -dr;
        }
        if (dg < 0)
        {
            gstep = -1;
            dg = -dg;
        }
        if (db < 0)
        {
            bstep = -1;
            db = -db;
        }
        int errorr = 2*dr - dx, errorg = 2*dg - dx, errorb = 2*db - dx;

        z = z0;
        for (int i = x1; i < x1tmp; i++)
            z += czx;

        for (int x = x1tmp; x <= x2tmp; x++)
        {
            if (dx > 0)
            {
                while (errorr > 0)
                {
                    r += rstep;
                    errorr -= 2*dx;
                }
                while (errorg > 0)
                {
                    g += gstep;
                    errorg -= 2*dx;
                }
                while (errorb > 0)
                {
                    b += bstep;
                    errorb -= 2*dx;
                }
            }

            if (z <= getDepth(x, y))
            {
                if (r < 0) r = 0;
                else if (r > 255) r = 255;
                if (g < 0) g = 0;
                else if (g > 255) g = 255;
                if (b < 0) b = 0;
                else if (b > 255) b = 255;
                drawPixel(pVram, x, y, RGBA(r,g,b,0));
                setDepth(x, y, z);
            }

            errorr += 2*dr;
            errorg += 2*dg;
            errorb += 2*db;
            z += czx;
        }

        error1 += 2*dx1;
        error2 += 2*dx2;
        errorr1 += 2*dr1;
        errorg1 += 2*dg1;
        errorb1 += 2*db1;
        errorr2 += 2*dr2;
        errorg2 += 2*dg2;
        errorb2 += 2*db2;
        z0 += czy;
    }

    // Top line

    int r = c1.R, g = c1.G, b = c1.B;
    int dr = c2.R - c1.R, dg = c2.G - c1.G, db = c2.B - c1.B;
    int dx = x2 - x1;
    int rstep = 1, gstep = 1, bstep = 1;
    if (dr < 0)
    {
        rstep = -1;
        dr = -dr;
    }
    if (dg < 0)
    {
        gstep = -1;
        dg = -dg;
    }
    if (db < 0)
    {
        bstep = -1;
        db = -db;
    }

    int errorr = 2*dr - dx, errorg = 2*dg - dx, errorb = 2*db - dx;
    z = z0;

    for (int x = x1; x <= x2; x++)
    {
        if (dx > 0)
        {
            while (errorr > 0)
            {
                r += rstep;
                errorr -= 2*dx;
            }
            while (errorg > 0)
            {
                g += gstep;
                errorg -= 2*dx;
            }
            while (errorb > 0)
            {
                b += bstep;
                errorb -= 2*dx;
            }
        }

        if (z <= getDepth(x, ytop))
        {
            if (r < 0)  r = 0;
            else if (r > 255)   r = 255;
            if (g < 0)  g = 0;
            else if (g > 255)   g = 255;
            if (b < 0)  b = 0;
            else if (b > 255)   b = 255;
            drawPixel(pVram, x, ytop, RGBA(r,g,b,0));
            setDepth(x, ytop, z);
        }

        errorr += 2*dr;
        errorg += 2*dg;
        errorb += 2*db;
        z += czx;
    }
}

void VideoBuffer::rasterize(Color* pVram, const Point3 &p1, const Point3 &p2, const Point3 &p3, RenderMode mode) 
{
    int x1 = (p1.x + 1)/2*(SCREEN_WIDTH-1), y1 = (p1.y + 1)/2*(SCREEN_HEIGHT-1);
    int x2 = (p2.x + 1)/2*(SCREEN_WIDTH-1), y2 = (p2.y + 1)/2*(SCREEN_HEIGHT-1);
    int x3 = (p3.x + 1)/2*(SCREEN_WIDTH-1), y3 = (p3.y + 1)/2*(SCREEN_HEIGHT-1);

    switch (mode)
    {
    case POINT_MODE:
        {
            break;
        }
    case FRAME_MODE:
        {
            drawLine(pVram, x1, y1, x2, y2, ColorWhite);
            drawLine(pVram, x2, y2, x3, y3, ColorWhite);
            drawLine(pVram, x3, y3, x1, y1, ColorWhite);
            break;
        }
    case NORMAL_MODE:
        {
            float z1 = p1.z, z2 = p2.z, z3 = p3.z;

            float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
            float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
            float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            float D = -(A*x1 + B*y1 + C*z1);

            if (y1 == y2)
            {
                if (y3 > y1)
                {
                    if (x1 <= x2)
                        drawTriangleFlatBottom(pVram, x3, y3, p3.color, x1, p1.color, x2, p2.color, y1, A, B, C, D);
                    else
                        drawTriangleFlatBottom(pVram, x3, y3, p3.color, x2, p2.color, x1, p1.color, y1, A, B, C, D);
                }
                else if (y3 < y1)
                {
                    if (x1 <= x2)
                        drawTriangleFlatTop(pVram, x3, y3, p3.color, x1, p1.color, x2, p2.color, y1, A, B, C, D);
                    else
                        drawTriangleFlatTop(pVram, x3, y3, p3.color, x2, p2.color, x1, p1.color, y1, A, B, C, D);
                }
            }
            else if (y1 == y3)
            {
                if (y2 > y1)
                {
                    if (x1 <= x3)
                        drawTriangleFlatBottom(pVram, x2, y2, p2.color, x1, p1.color, x3, p3.color, y1, A, B, C, D);
                    else
                        drawTriangleFlatBottom(pVram, x2, y2, p2.color, x3, p3.color, x1, p1.color, y1, A, B, C, D);
                }
                else if (y2 < y1)
                {
                    if (x1 <= x3)
                        drawTriangleFlatTop(pVram, x2, y2, p2.color, x1, p1.color, x3, p3.color, y1, A, B, C, D);
                    else
                        drawTriangleFlatTop(pVram, x2, y2, p2.color, x3, p3.color, x1, p1.color, y1, A, B, C, D);
                }
            }
            else if (y2 == y3)
            {
                if (y1 > y2)
                {
                    if (x2 <= x3)
                        drawTriangleFlatBottom(pVram, x1, y1, p1.color, x2, p2.color, x3, p3.color, y2, A, B, C, D);
                    else
                        drawTriangleFlatBottom(pVram, x1, y1, p1.color, x3, p3.color, x2, p2.color, y2, A, B, C, D);
                }
                else if (y1 < y2)
                {
                    if (x2 <= x3)
                        drawTriangleFlatTop(pVram, x1, y1, p1.color, x2, p2.color, x3, p3.color, y2, A, B, C, D);
                    else
                        drawTriangleFlatTop(pVram, x1, y1, p1.color, x3, p3.color, x2, p2.color, y2, A, B, C, D);
                }
            }
            else
            {
                int xtop, ytop, xmid, ymid, xbtm, ybtm;
                Color ctop, cmid, cbtm;
                if (y1 > y2 && y2 > y3) // y1 y2 y3
                {
                    xtop = x1;
                    ytop = y1;
                    ctop = p1.color;
                    xmid = x2;
                    ymid = y2;
                    cmid = p2.color;
                    xbtm = x3;
                    ybtm = y3;
                    cbtm = p3.color;
                }
                else if (y1 > y3 && y3 > y2) // y1 y3 y2
                {
                    xtop = x1;
                    ytop = y1;
                    ctop = p1.color;
                    xmid = x3;
                    ymid = y3;
                    cmid = p3.color;
                    xbtm = x2;
                    ybtm = y2;
                    cbtm = p2.color;
                }
                else if (y2 > y1 && y1 > y3) // y2 y1 y3
                {
                    xtop = x2;
                    ytop = y2;
                    ctop = p2.color;
                    xmid = x1;
                    ymid = y1;
                    cmid = p1.color;
                    xbtm = x3;
                    ybtm = y3;
                    cbtm = p3.color;
                }
                else if (y2 > y3 && y3 > y1) // y2 y3 y1
                {
                    xtop = x2;
                    ytop = y2;
                    ctop = p2.color;
                    xmid = x3;
                    ymid = y3;
                    cmid = p3.color;
                    xbtm = x1;
                    ybtm = y1;
                    cbtm = p1.color;
                }
                else if (y3 > y1 && y1 > y2) // y3 y1 y2
                {
                    xtop = x3;
                    ytop = y3;
                    ctop = p3.color;
                    xmid = x1;
                    ymid = y1;
                    cmid = p1.color;
                    xbtm = x2;
                    ybtm = y2;
                    cbtm = p2.color;
                }
                else if (y3 > y2 && y2 > y1) // y3 y2 y1
                {
                    xtop = x3;
                    ytop = y3;
                    ctop = p3.color;
                    xmid = x2;
                    ymid = y2;
                    cmid = p2.color;
                    xbtm = x1;
                    ybtm = y1;
                    cbtm = p1.color;
                }

                int xmid2 = (ymid-ybtm) * (xbtm-xtop) / (ybtm-ytop) + xbtm + 0.5;
                int rmid2 = (ymid-ybtm) * (cbtm.R-ctop.R) / (ybtm-ytop) + cbtm.R + 0.5;
                int gmid2 = (ymid-ybtm) * (cbtm.G-ctop.G) / (ybtm-ytop) + cbtm.G + 0.5;
                int bmid2 = (ymid-ybtm) * (cbtm.B-ctop.B) / (ybtm-ytop) + cbtm.B + 0.5;
                if (xmid2 < xmid)
                {
                    drawTriangleFlatBottom(pVram, xtop, ytop, ctop, xmid2, RGB(rmid2,gmid2,bmid2), xmid, cmid, ymid, A, B, C, D);
                    drawTriangleFlatTop(pVram, xbtm, ybtm, cbtm, xmid2, RGB(rmid2,gmid2,bmid2), xmid, cmid, ymid, A, B, C, D);
                }
                else
                {
                    drawTriangleFlatBottom(pVram, xtop, ytop, ctop, xmid, cmid, xmid2, RGB(rmid2,gmid2,bmid2), ymid, A, B, C, D);
                    drawTriangleFlatTop(pVram, xbtm, ybtm, cbtm, xmid, cmid, xmid2, RGB(rmid2,gmid2,bmid2), ymid, A, B, C, D);
                }
            }
            break;
        }
    }
}

void VideoBuffer::Render(vector<Point3>& points, vector<unsigned int>& indices, RenderMode mode)
{
    Color* pVram = getVramDisplayBuffer();
    memset(pVram, 0, sizeof(int)*BUF_WIDTH*SCREEN_HEIGHT);
    ClearDepth();
    for (int i = 0; i < indices.size(); i += 3)
    {
        rasterize(pVram, points[indices[i]], points[indices[i+1]], points[indices[i+2]], mode);
    }
#if DEBUG
    fps((int)pVram);
#endif
    sceDisplayWaitVblankStart();
    sceDisplaySetFrameBuf(pVram, BUF_WIDTH, PSP_DISPLAY_PIXEL_FORMAT_8888, 0);
    flipBuffer();
}

void GetViewTransformation(const Point3 &eye, const Point3 &interest, const Vector3 &up, Matrix4& mat)
{
    Matrix4 T = trans_mat(-(eye - Point3(0, 0, 0)));

    Vector3 w = normalize(interest - eye);
    Vector3 u = normalize(cross(w, up));
    Vector3 v = cross(u, w);
    Matrix4 R(
        u.x,  u.y,  u.z,  0,
        v.x,  v.y,  v.z,  0,
        -w.x, -w.y, -w.z, 0,
        0,    0,    0,    1);

    Matrix4 Z;
    Z.m[10] = -1;

    mat = Z * R * T;
}

void GetPerspectiveTransformation(float xfov, float aspectRatio, float znear, float zfar, Matrix4& mat)
{
    float W, H;
    W = znear * tan(DegreeToRadian(xfov/2)) * 2;
    H = W / aspectRatio;

    mat.m[0] = (2 * znear) / W;
    mat.m[5] = (2 * znear) / H;
    mat.m[10] = zfar / (zfar - znear);
    mat.m[11] = 1;
    mat.m[14] = -(zfar * znear) / (zfar - znear);
    mat.m[15] = 0;
}

inline void AddTwoPoint(const Point3 &pNew1, const Point3 &pNew2, vector<Point3>* points)
{
    points->push_back(pNew1);
    points->push_back(pNew2);
}

bool IsPointInside(CLIP side, const Point3 &p, double w)
{
    double x = p.x / w;
    double y = p.y / w;
    double z = p.z / w;

    switch (side)
    {
    case HITHER:
        {
            if (z < 0 || w < 0)
                return false;
            break;
        }
    case YON:
        {
            if (z > 1)
                return false;
            break;
        }
    case LEFT:
        {
            if (x < -1)
                return false;
            break;
        }
    case RIGHT:
        {
            if (x > 1)
                return false;
            break;
        }
    case BOTTOM:
        {
            if (y < -1)
                return false;
            break;
        }
    case TOP:
        {
            if (y > 1)
                return false;
            break;
        }
    }
    return true;
}

Point3 CalcIntersection(const Point3 p1, const Point3 p2, double w1, double w2, CLIP side)
{
    Point3 point;
    double x1 = p1.x / w1;
    double y1 = p1.y / w1;
    double z1 = p1.z / w1;
    double x2 = p2.x / w2;
    double y2 = p2.y / w2;
    double z2 = p2.z / w2;
    double t;

    switch (side)
    {
    case HITHER:
        {
            t = z1 / (z1 - z2);
            point.x = x1 + (x2 - x1) * t;
            point.y = y1 + (y2 - y1) * t;
            point.z = 0;
            break;
        }
    case YON:
        {
            t = (z1 - 1) / (z1 - z2);
            point.x = x1 + (x2 - x1) * t;
            point.y = y1 + (y2 - y1) * t;
            point.z = 1;
            break;
        }
    case LEFT:
        {
            t = (x1 + 1) / (x1 - x2);
            point.x = -1;
            point.y = y1 + (y2 - y1) * t;
            point.z = z1 + (z2 - z1) * t;
            break;
        }
    case RIGHT:
        {
            t = (x1 - 1) / (x1 - x2);
            point.x = 1;
            point.y = y1 + (y2 - y1) * t;
            point.z = z1 + (z2 - z1) * t;
            break;
        }
    case BOTTOM:
        {
            t = (y1 + 1) / (y1 - y2);
            point.x = x1 + (x2 - x1) * t;
            point.y = -1;
            point.z = z1 + (z2 - z1) * t;
            break;
        }
    case TOP:
        {
            t = (y1 - 1) / (y1 - y2);
            point.x = x1 + (x2 - x1) * t;
            point.y = 1;
            point.z = z1 + (z2 - z1) * t;
            break;
        }
    }
    point.color.R = p1.color.R + (p2.color.R - p1.color.R) * t;
    point.color.G = p1.color.G + (p2.color.G - p1.color.G) * t;
    point.color.B = p1.color.B + (p2.color.B - p1.color.B) * t;
    point.color.A = p1.color.A + (p2.color.A - p1.color.A) * t;

    return point;
}

void Clip(CLIP clipSide, vector<Point3> *points, vector<unsigned int> *indices)
{
    Point3 p1, p2, p3;
    double w1, w2, w3;
    Point3 newP1, newP2, newP3;

    bool sss = false;

    int iEnd = indices->size();
    for (int i = 0; i < iEnd; i += 3)
    {
        p1 = (*points)[(*indices)[i]];
        w1 = p1.w;
        p2 = (*points)[(*indices)[i + 1]];
        w2 = p2.w;
        p3 = (*points)[(*indices)[i + 2]];
        w3 = p3.w;

        if (IsPointInside(clipSide, p1, w1))
        {
            // P1 in
            if (IsPointInside(clipSide, p2, w2))
            {
                // P2 in
                if (IsPointInside(clipSide, p3, w3))
                {
                    // P3 in -> keep
                    continue;
                }
                else
                {
                    // P3 out -> replace and add
                    newP1 = CalcIntersection(p1, p3, w1, w3, clipSide);
                    newP2 = CalcIntersection(p2, p3, w2, w3, clipSide);
                    AddTwoPoint(newP1, newP2, points);
                    (*indices)[i + 2] = points->size() - 1;
                    indices->push_back((*indices)[i]);
                    indices->push_back(points->size() - 1);
                    indices->push_back(points->size() - 2);
                    iEnd += 3;
                }
            }
            else
            {
                // P2 out
                if (IsPointInside(clipSide, p3, w3))
                {
                    // P3 in -> replace and add
                    newP1 = CalcIntersection(p1, p2, w1, w2, clipSide);
                    newP3 = CalcIntersection(p3, p2, w3, w2, clipSide);
                    AddTwoPoint(newP1, newP3, points);
                    (*indices)[i + 1] = points->size() - 2;
                    indices->push_back(points->size() - 2);
                    indices->push_back(points->size() - 1);
                    indices->push_back((*indices)[i + 2]);
                    iEnd += 3;
                }
                else
                {
                    // P3 out -> replace
                    newP2 = CalcIntersection(p1, p2, w1, w2, clipSide);
                    newP3 = CalcIntersection(p1, p3, w1, w3, clipSide);
                    AddTwoPoint(newP2, newP3, points);
                    (*indices)[i + 1] = points->size() - 2;
                    (*indices)[i + 2] = points->size() - 1;
                }
            }
        }
        else
        {
            // P1 out
            if (IsPointInside(clipSide, p2, w2))
            {
                // P2 in
                if (IsPointInside(clipSide, p3, w3))
                {
                    // P3 in -> replace and add
                    newP2 = CalcIntersection(p1, p2, w1, w2, clipSide);
                    newP3 = CalcIntersection(p1, p3, w1, w3, clipSide);
                    AddTwoPoint(newP2, newP3, points);
                    (*indices)[i] = points->size() - 1;
                    indices->push_back((*indices)[i + 1]);
                    indices->push_back(points->size() - 1);
                    indices->push_back(points->size() - 2);
                    iEnd += 3;
                }
                else
                {
                    // P3 out -> replace
                    newP1 = CalcIntersection(p1, p2, w1, w2, clipSide);
                    newP3 = CalcIntersection(p3, p2, w3, w2, clipSide);
                    AddTwoPoint(newP1, newP3, points);
                    (*indices)[i] = points->size() - 2;
                    (*indices)[i + 2] = points->size() - 1;
                }
            }
            else
            {
                // P2 out
                if (IsPointInside(clipSide, p3, w3))
                {
                    // P3 in -> replace
                    newP1 = CalcIntersection(p1, p3, w1, w3, clipSide);
                    newP2 = CalcIntersection(p2, p3, w2, w3, clipSide);
                    AddTwoPoint(newP1, newP2, points);
                    (*indices)[i] = points->size() - 2;
                    (*indices)[i + 1] = points->size() - 1;
                }
                else
                {
                    // P3 out -> kick out
                    (*indices)[i] = (*indices)[iEnd - 3];
                    (*indices)[i + 1] = (*indices)[iEnd - 2];
                    (*indices)[i + 2] = (*indices)[iEnd - 1];
                    indices->pop_back();
                    indices->pop_back();
                    indices->pop_back();
                    iEnd -= 3;
                    i -= 3;
                }
            }
        }
    }
}

// points are the clipped ones
void Cull(vector<Point3> *points, vector<unsigned int> *indices)
{
    Point3 p1, p2, p3;
    int iEnd = indices->size();
    for (int i = 0; i < iEnd; i += 3)
    {
        p1 = (*points)[(*indices)[i]];
        p2 = (*points)[(*indices)[i + 1]];
        p3 = (*points)[(*indices)[i + 2]];
        double z = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        if (z < 0)
        {
            (*indices)[i] = (*indices)[iEnd - 3];
            (*indices)[i + 1] = (*indices)[iEnd - 2];
            (*indices)[i + 2] = (*indices)[iEnd - 1];
            indices->pop_back();
            indices->pop_back();
            indices->pop_back();
            iEnd -= 3;
            i -= 3;
        }
    }
}
