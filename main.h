//**************************************************************************
//		PSPGU Tutorial: 'Lesson1' - main.h
//**************************************************************************
// Includes
#include <malloc.h>		//For memalign()
#include <pspkernel.h>
#include <pspdisplay.h>
#include <pspdebug.h>
#include <stdio.h>

//**************************************************************************
int main(int argc, char **argv); 

// Exit callback
int ExitCallback(int arg1, int arg2, void *common) {
	sceKernelExitGame();
	return 0;
}
// Callback thread 
int ThreadFunc(SceSize args, void *argp) {
	int cbid;

	cbid = sceKernelCreateCallback("Exit Callback", ExitCallback, NULL);
	sceKernelRegisterExitCallback(cbid);

	sceKernelSleepThreadCB();

	return 0;
}
// Sets up the callback thread and returns its thread id
int SetupCallbacks(void) {
	int thid = 0;

	thid = sceKernelCreateThread("update_thread", ThreadFunc, 0x11, 0xFA0, 0, 0);
	if(thid >= 0) {
		sceKernelStartThread(thid, 0, 0);
	}

	return thid;
}

#include <pspgu.h>
#include <pspgum.h>
#include <psprtc.h>				// for the timer/fps functions

#define BUF_WIDTH (512)
#define SCR_WIDTH (480)
#define SCR_HEIGHT (272)

typedef struct 
{
	unsigned int color;
	float x, y, z;
} Vertex;

void FPS(void);				// Display Frames Per Second 

//Graphics related functions:
void InitGU(void);			        // Initialize the Graphics Subsystem
void SetupProjection(void);	                // Setups the Projection Matrix
void DrawScene(void);			        // Render Geometry
