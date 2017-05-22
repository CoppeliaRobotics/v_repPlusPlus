// Copyright 2016 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#ifndef VREPPLUSPLUS_PLUGIN_H_INCLUDED
#define VREPPLUSPLUS_PLUGIN_H_INCLUDED

#include <iostream>

#include "v_repLib.h"

#ifdef _WIN32
	#define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#define _stricmp strcasecmp
#endif /* __linux || __APPLE__ */

namespace vrep
{
    class Plugin
    {
    public:
        virtual void onStart();
        virtual void onEnd();
        virtual void * onMessage(int message, int *auxData, void *customData, int *replyData);
        virtual LIBRARY loadVrepLibrary();

        virtual void onInstancePass(bool objectsErased, bool objectsCreated, bool modelLoaded, bool sceneLoaded, bool undoCalled, bool redoCalled, bool sceneSwitched, bool editModeActive, bool objectsScaled, bool selectionStateChanged, bool keyPressed, bool simulationStarted, bool simulationEnded);
        virtual void onInstanceSwitch(int sceneID);
        virtual void onInstanceAboutToSwitch(int sceneID);
        virtual void onMenuItemSelected(int itemHandle, int itemState);
        virtual void onBroadcast(int header, int messageID);
        virtual void onSceneSave();
        virtual void onModelSave();
        virtual void onModuleOpen(char *name);
        virtual void onModuleHandle(char *name);
        virtual void onModuleHandleInSensingPart(char *name);
        virtual void onModuleClose(char *name);
        virtual void onRenderingPass();
        virtual void onBeforeRendering();
        virtual void onImageFilterEnumReset();
        virtual void onImageFilterEnumerate(int &headerID, int &filterID, std::string &name);
        virtual void onImageFilterAdjustParams(int headerID, int filterID, int bufferSize, void *buffer, int &editedBufferSize, void *&editedBuffer);
        virtual std::vector<simFloat> onImageFilterProcess(int headerID, int filterID, int resX, int resY, int visionSensorHandle, simFloat *inputImage, simFloat *depthImage, simFloat *workImage, simFloat *bufferImage1, simFloat *bufferImage2, simFloat *outputImage, void *filterParamBuffer, int &triggerDetectionn);
        virtual void onAboutToUndo();
        virtual void onUndo();
        virtual void onAboutToRedo();
        virtual void onRedo();
        virtual void onScriptIconDblClick(int objectHandle, int &dontOpenEditor);
        virtual void onSimulationAboutToStart();
        virtual void onSimulationAboutToEnd();
        virtual void onSimulationEnded();
        virtual void onKeyPress(int key, int mods);
        virtual void onBannerClicked(int bannerID);
        virtual void onRefreshDialogs(int refreshDegree);
        virtual void onSceneLoaded();
        virtual void onModelLoaded();
        virtual void onGuiPass();
        virtual void onMainScriptAboutToBeCalled(int &out);
        virtual void onRMLPos();
        virtual void onRMLVel();
        virtual void onRMLStep();
        virtual void onRMLRemove();
        virtual void onPathPlanningPlugin();
        virtual void onColladaPlugin();
        virtual void onOpenGL(int programIndex, int renderingAttributes, int cameraHandle, int viewIndex);
        virtual void onOpenGLFrame(int sizeX, int sizeY, int &out);
        virtual void onOpenGLCameraView(int sizeX, int sizeY, int viewIndex, int &out);
        virtual void onProxSensorSelectDown(int objectID, simFloat *clickedPoint, simFloat *normalVector);
        virtual void onProxSensorSelectUp(int objectID, simFloat *clickedPoint, simFloat *normalVector);
        virtual void onPickSelectDown(int objectID);

    };
}

#define VREP_PLUGIN(pluginName, pluginVersion, className) \
LIBRARY vrepLib; \
className vrepPlugin; \
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt) \
{ \
    try \
    { \
        vrepLib = vrepPlugin.loadVrepLibrary(); \
        vrepPlugin.onStart(); \
        return pluginVersion; \
    } \
    catch(std::exception &ex) \
    { \
        std::cout << pluginName << ": " << ex.what() << std::endl; \
        return 0; \
    } \
} \
VREP_DLLEXPORT void v_repEnd() \
{ \
    vrepPlugin.onEnd(); \
    unloadVrepLibrary(vrepLib); \
} \
VREP_DLLEXPORT void * v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData) \
{ \
    return vrepPlugin.onMessage(message, auxiliaryData, customData, replyData); \
}

#endif // VREPPLUSPLUS_PLUGIN_H_INCLUDED
