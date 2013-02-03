#include <pspkernel.h>
#include <pspdebug.h>
#include <pspdisplay.h>
#include <pspctrl.h>
#include "def.h"
#include "pipeline.h"
#include "primitive.h"

PSP_MODULE_INFO("HELLO WORLD", 0, 1, 1);
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER);

volatile int end_flag = 0;
VideoBuffer* buf = VideoBuffer::GetInstance();

int ExitCallback(int arg1, int arg2, void *common)
{
    end_flag = 1;
    return 0;
}

int ThreadFunc(SceSize args, void *argp)
{
    int cbid = sceKernelCreateCallback("exit callback", ExitCallback, NULL);
    sceKernelRegisterExitCallback(cbid);

    sceKernelSleepThreadCB();

    return 0;
}

int main(int argc, char *argv[])
{
    int thid = sceKernelCreateThread("update_thread", ThreadFunc, 0x11, 0xFA0, 0, 0);
    if (thid >= 0)
        sceKernelStartThread(thid, 0, 0);

    pspDebugScreenInit();
    SceCtrlData pad;
    sceCtrlSetSamplingCycle(0);
    sceCtrlSetSamplingMode(PSP_CTRL_MODE_ANALOG);
    InitiateGraphicEngine(); 

    Cube cube(10);
    vector<Point3> points;
    //vector<Vector3> normals;
    vector<unsigned int> indices;

    Point3 eye(20,20,20), lookat(0,0,0);
    Vector3 lookup(0,1,0);
    Vector3 eyemoveLR = normalize(cross((lookat - eye), lookup));
    Vector3 eyemoveFB = normalize(lookat - eye);
    float pitch = 0;

    Matrix4 matV, matP, mat;
    GetPerspectiveTransformation(60, AspectRatio, 20, 200, matP);

    bool camUpdate = true;

    while (!end_flag)
    {
        sceCtrlReadBufferPositive(&pad, 1);

        if(pad.Buttons & PSP_CTRL_LEFT)
        {
            eye = eye - eyemoveLR;
            lookat = lookat - eyemoveLR;
            camUpdate = true;
        }
        else if(pad.Buttons & PSP_CTRL_RIGHT)
        {
            eye = eye + eyemoveLR;
            lookat = lookat + eyemoveLR;
            camUpdate = true;
        }

        if(pad.Buttons & PSP_CTRL_UP)
        {
            eye = eye + eyemoveFB;
            lookat = lookat + eyemoveFB;
            camUpdate = true;
        }
        else if(pad.Buttons & PSP_CTRL_DOWN)
        {
            eye = eye - eyemoveFB;
            lookat = lookat - eyemoveFB;
            camUpdate = true;
        }

        if(pad.Ly < 64)
        {
            if (pitch < 89)
            {
                pitch++;
                Matrix4 matR = rot_mat(eye, eyemoveLR, DegreeToRadian(1));
                lookat = matR * lookat;
            }
            camUpdate = true;
        }
        else if(pad.Ly > 192)
        {
            if (pitch > -89)
            {
                pitch--;
                Matrix4 matR = rot_mat(eye, eyemoveLR, DegreeToRadian(-1));
                lookat = matR * lookat;
            }
            camUpdate = true;
        }

        if(pad.Lx < 64)
        {
            Vector3 up = cross(eyemoveLR, eyemoveFB);
            Matrix4 matR = rot_mat(eye, up, DegreeToRadian(1));
            lookat = matR * lookat;
            eyemoveLR = normalize(cross(eyemoveFB, lookup));
            camUpdate = true;
        }
        else if(pad.Lx > 192)
        {
            Vector3 up = cross(eyemoveLR, eyemoveFB);
            Matrix4 matR = rot_mat(eye, up, DegreeToRadian(-1));
            lookat = matR * lookat;
            eyemoveLR = normalize(cross(eyemoveFB, lookup));
            camUpdate = true;
        }
        
        if (camUpdate)
        {
            // Recalculate points
            GetViewTransformation(eye, lookat, lookup, matV);
            mat = matP * matV;
            cube.CopyPoints(&points);
            cube.CopyIndices(&indices);
            for (int i = 0; i < points.size(); i++)
            {
                mat_times_point(mat, &points[i], &points[i].w);
                if (points[i].w == 0)
                    points[i].w = EPSILON;
            }
            Clip(HITHER, &points, &indices);
            Clip(YON, &points, &indices);
            Clip(LEFT, &points, &indices);
            Clip(RIGHT, &points, &indices);
            Clip(TOP, &points, &indices);
            Clip(BOTTOM, &points, &indices);
            for (int i = 0; i < points.size(); i++)
            {
                points[i].x /= points[i].w;
                points[i].y /= points[i].w;
                points[i].z /= points[i].w;
            }
            Cull(&points, &indices);
            camUpdate = false;
        }        

        if(pad.Buttons & PSP_CTRL_TRIANGLE)
        {
            buf->Render(points, indices, FRAME_MODE);
        }
        else
        {
            buf->Render(points, indices, NORMAL_MODE);
        }
    }

    sceKernelExitGame();

    return 0;
}
