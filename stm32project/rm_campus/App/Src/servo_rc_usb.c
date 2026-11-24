#include "servo_rc_usb.h"
#if SERVO_RC_ENABLE && SERVO_BUS_ENABLE && SERVO_RC_TELEMETRY_ENABLE
#include "servo_rc.h"
#include "arm_usb.h"
#include "servo_app.h"
#include "servo_usb.h"
#include "motor_status_usb.h"
#include "flash_tasks.h"
#include "usb_serial.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#if FLASH_TASKS_ENABLE
#error "Disable SERVO_RC_TELEMETRY_ENABLE before enabling Flash tasks"
#endif
#if SERVO_RC_TELEMETRY_MS == 0
#error "Servo telemetry period must be positive"
#endif
static bool pending,owned,active;
static uint32_t session,last_ms;
static UsbSerialTxTicket ticket;
bool ServoRcUsb_Selected(void) { return ServoRc_Selected(HAL_GetTick()); }
bool ServoRcUsb_Busy(void) { return pending || owned; }
static void release(void)
{
    if(owned)FlashTasks_ReleaseUsb(FLASH_USB_SERVO_TELEMETRY);
    pending=owned=false;
}
void ServoRcUsb_Process(void)
{
    UsbSerialStatus usb;UsbSerial_GetStatus(&usb);
    if(!usb.ready || session!=usb.session_id) { release();active=false;session=usb.session_id; }
    if(pending) {
        UsbSerialTxState tx=UsbSerial_GetTxState(&ticket);
        if(tx==USB_TX_PENDING)return;
        release();
        if(tx!=USB_TX_COMPLETE)active=false;
    }
    if(ArmUsb_Busy()) { active=false;return; }
    uint32_t now=HAL_GetTick();
    if(!usb.ready || !ServoRc_Selected(now)) { active=false;return; }
    if(!active) { active=true;last_ms=now;return; }
    if((uint32_t)(now-last_ms)<SERVO_RC_TELEMETRY_MS)return;
    last_ms=now; /* No backlog/catch-up: each row uses the latest measured positions. */
    if(MotorStatusUsb_Busy() || ServoUsb_Busy() || usb.tx_buffered)return;
    ServoSnapshot snapshot;Servo_CopySnapshot(&snapshot);
    int values[5]={-1000,-1000,-1000,-1000,-1000};
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        const ServoState *s=&snapshot.device[i];
        if(s->id<1U || s->id>5U || !s->online || !s->position_seen ||
            (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS || !s->mode_valid ||
            s->mode<1U || s->mode>4U || s->position<500U || s->position>2500U)continue;
        /* Protocol angle: P500=0 degrees. Never substitute a requested target. */
        values[s->id-1U]=(int)((uint32_t)(s->position-500U)*(s->mode<=2U ? 135U : 90U));
    }
    char frame[64];unsigned used=0;
    for(unsigned i=0;i<5;++i) {
        unsigned mag=(unsigned)(values[i]<0 ? -values[i] : values[i]);
        int n=snprintf(frame+used,sizeof(frame)-used,"%s%u.%03u%s",values[i]<0?"-":"",mag/1000U,mag%1000U,i==4?"\n":",");
        if(n<0 || (unsigned)n>=sizeof(frame)-used)return;
        used+=(unsigned)n;
    }
    if(!FlashTasks_AcquireUsb(FLASH_USB_SERVO_TELEMETRY))return;
    owned=true;
    taskENTER_CRITICAL();
    UsbSerial_GetStatus(&usb);
    if(usb.ready && usb.session_id==session && ServoRc_Selected(HAL_GetTick()) && !ArmUsb_Busy() &&
        UsbSerial_SendTracked(frame,(uint16_t)used,&ticket)==USB_SERIAL_OK)pending=true;
    taskEXIT_CRITICAL();
    if(!pending)release();
}
#endif
