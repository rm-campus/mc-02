#include "arm_usb.h"
#if ARM_VELOCITY_IK_ENABLE
#include "arm_velocity.h"
#include "flash_tasks.h"
#include "servo_usb.h"
#include "motor_status_usb.h"
#include "usb_serial.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
static bool pending,owned,active;
static uint32_t session,last_ms,fault_sequence;
static UsbSerialTxTicket ticket;
bool ArmUsb_Busy(void)
{
    ArmStatus s;Arm_GetStatus(&s);
    return s.state==ARM_FAULT || pending || owned;
}
static void release(void)
{
    if(owned)FlashTasks_ReleaseUsb(FLASH_USB_ARM_ERROR);
    owned=pending=false;
}
/* 不依赖newlib-nano浮点printf；异常值显示NA，避免数值转换溢出。 */
static void decimal(char out[24],float value)
{
    if(!isfinite(value) || fabsf(value)>1000000.0f) { (void)snprintf(out,24,"NA");return; }
    long scaled=lroundf(value*1000.0f);unsigned long mag=(unsigned long)(scaled<0?-scaled:scaled);
    (void)snprintf(out,24,"%s%lu.%03lu",scaled<0?"-":"",mag/1000U,mag%1000U);
}
void ArmUsb_Process(void)
{
    UsbSerialStatus usb;UsbSerial_GetStatus(&usb);
    if(!usb.ready || session!=usb.session_id) { release();active=false;session=usb.session_id; }
    if(pending) {
        UsbSerialTxState tx=UsbSerial_GetTxState(&ticket);
        if(tx==USB_TX_PENDING)return;
        release();
    }
    ArmStatus s;Arm_GetStatus(&s);
    if(!usb.ready || s.state!=ARM_FAULT) { active=false;return; }
    uint32_t now=HAL_GetTick();
    if(active && fault_sequence==s.fault_sequence && (uint32_t)(now-last_ms)<ARM_ERROR_PERIOD_MS)return;
    if(ServoUsb_Busy() || MotorStatusUsb_Busy() || usb.tx_buffered)return;
    const char *unit="none";
    if(s.error==ARM_ERROR_JOINT_SPEED_LIMIT)unit="deg_s";
    else if(s.error==ARM_ERROR_LINEAR_SPEED_LIMIT || s.error==ARM_ERROR_VELOCITY_UNACHIEVABLE)unit="mm_s";
    else if(s.error==ARM_ERROR_JOINT_LIMIT || s.error==ARM_ERROR_TRACKING)unit="deg";
    else if(s.error==ARM_ERROR_TIMING || s.error==ARM_ERROR_COMMAND_TIMEOUT || s.error==ARM_ERROR_FEEDBACK_STALE || s.error==ARM_ERROR_FEEDBACK_SKEW)unit="ms";
    char requested[24],limit[24],linear[24],angular[24],frame[256];
    decimal(requested,s.requested);decimal(limit,s.limit);decimal(linear,s.linear_residual_m_s*1000.0f);decimal(angular,ARM_RAD_TO_DEG(s.angular_residual_rad_s));
    int n=snprintf(frame,sizeof(frame),"ARM_ERROR code=%s joint=%u servo_id=%u requested=%s limit=%s unit=%s linear_residual_mm_s=%s angular_residual_deg_s=%s\n",
        Arm_ErrorName(s.error),s.joint,s.servo_id,requested,limit,unit,linear,angular);
    if(n<=0 || n>=(int)sizeof(frame) || !FlashTasks_AcquireUsb(FLASH_USB_ARM_ERROR))return;
    owned=true;
    taskENTER_CRITICAL();
    ArmStatus latest;Arm_GetStatus(&latest);UsbSerial_GetStatus(&usb);
    if(usb.ready && usb.session_id==session && latest.state==ARM_FAULT && latest.fault_sequence==s.fault_sequence &&
        UsbSerial_SendTracked(frame,(uint16_t)n,&ticket)==USB_SERIAL_OK) {
        pending=active=true;last_ms=now;fault_sequence=s.fault_sequence;
    }
    taskEXIT_CRITICAL();
    if(!pending)release();
}
#endif
