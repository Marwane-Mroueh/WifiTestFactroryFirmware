#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

    /** Robot wide IPC bus. */
    extern messagebus_t bus;

    extern parameter_namespace_t parameter_root;
    bool get_demo15_state_1(void);
    bool get_one_time(void);

#ifdef __cplusplus
}
#endif

#endif
