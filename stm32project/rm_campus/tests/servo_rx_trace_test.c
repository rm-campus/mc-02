/* 真实的驱动、解析器和 USB 输出；从 HAL 接收回调注入字节，毫秒时钟可控。 */
#define main full_fixture_main
#include "servo_test.c"
#undef main

static void rx(uint8_t b)
{
    assert(rx_byte);*rx_byte=b;rx_byte=NULL;HAL_UART_RxCpltCallback(&huart7);
}
static void usb_ticks(unsigned n)
{
    while(n--) { ++tick;Campus_UsbProcess(); }
}
static void raw(void)
{
    const uint8_t bytes[]={0,0xff,'\r','\n','x','#','!'};
    tick=10;assert(ServoUart_Init());
    for(unsigned i=0;i<sizeof(bytes);++i)rx(bytes[i]);
    assert(ServoUart_Recover()); /* 故意清除协议环，电脑仍能看到恢复前的散字节。 */
    rx(0x81);usb.ready=true;usb_ticks(12);
    assert(strstr(output,"seq=0 n=7 first_ms=10 last_ms=10 tx=NA tc_ms=NA dt_ms=NA lost=0 HEX=00 FF 0D 0A 78 23 21 ASCII=....x#!\n"));
    assert(strstr(output,"seq=7 n=1 ") && strstr(output,"HEX=81 ASCII=.\n"));
    uint8_t b;assert(ServoUart_Read(&b,1)==1 && b==0x81); /* USB 消费不影响驱动消费。 */
}
static void overflow(void)
{
    assert(ServoUart_Init());
    for(unsigned i=0;i<SERVO_TRACE_BLOCKS+7U;++i) {
        tick+=3;rx((uint8_t)i);
        uint8_t b;assert(ServoUart_Read(&b,1) && b==i);
    }
    usb.ready=true;usb_ticks(SERVO_TRACE_BLOCKS*5U);
    char *line=output;
    for(unsigned i=7;i<SERVO_TRACE_BLOCKS+7U;++i) {
        unsigned seq,byte;
        char *hex=strstr(line,"HEX=");assert(hex);
        assert(sscanf(line,"SERVO_RX seq=%u n=1",&seq)==1 && seq==i);
        assert(sscanf(hex,"HEX=%x",&byte)==1 && byte==i);
        char *lost=strstr(line,i==7?" lost=7 ":" lost=0 ");assert(lost && lost<hex);
        line=strchr(hex,'\n')+1;
    }
    assert(*line==0);
}
static void reconnect(void)
{
    assert(ServoUart_Init());
    usb.ready=true;usb_hold=true;rx('A');usb_ticks(8);
    assert(usb_len && !output_len);
    usb.ready=false;usb_len=0;++usb.session_id;usb_ticks(3);
    rx('B');usb_hold=false;usb.ready=true;usb_ticks(15);
    char *a=strstr(output,"HEX=41 ASCII=A\n"), *b=strstr(output,"HEX=42 ASCII=B\n");
    assert(a && b && a<b && strstr(output,"seq=0 n=1 ") && strstr(output,"seq=1 n=1 "));
}
static void burst(void)
{
    assert(ServoUart_Init());
    /* 原协议环剩一空位。触发其溢出的最后一字节也必须已经进入监听环。 */
    for(unsigned i=0;i<SERVO_RX_RING_SIZE;++i)rx((uint8_t)i);
    ServoUartStatus status;ServoUart_CopyStatus(&status);
    assert(status.overflows==1 && status.rx_bytes==SERVO_RX_RING_SIZE);
    assert(ServoUart_Recover());
    usb.ready=true;usb_ticks(60);
    char *line=output;
    for(unsigned seen=0;seen<SERVO_RX_RING_SIZE;) {
        unsigned offset,count;
        assert(sscanf(line,"SERVO_RX seq=%u n=%u",&offset,&count)==2 && offset==seen);
        assert(count && count<=SERVO_TRACE_BYTES && seen+count<=SERVO_RX_RING_SIZE);
        char *hex=strstr(line," lost=0 HEX=");assert(hex);
        hex+=strlen(" lost=0 HEX=");
        for(unsigned i=0;i<count;++i) {
            unsigned value;assert(sscanf(hex,"%2x",&value)==1 && value==(uint8_t)seen);
            ++seen;hex+=3;
        }
        line=strchr(hex,'\n')+1;
    }
    assert(!*line);
}
static void commands(void)
{
    field_reply_mask=7U; /* 001～003 复现截图，其余两台沿用手册格式，同一条总线混用。 */
    usb.ready=true;assert(Servo_Init());run(400);assert(!moves && strstr(output,"SERVO_RX "));
    assert(strstr(output,"ASCII=#001@ ZL-ZServo_AD_CBM V2.1.16STG!..\n"));
    assert(strstr(output,"ASCII=#001MOD1!\n"));
    ServoSnapshot snapshot;Servo_CopySnapshot(&snapshot);
    assert(!snapshot.malformed && !snapshot.unexpected);
    for(unsigned id=1;id<=5;++id) {
        ServoState s=state((uint8_t)id);
        assert(s.online && s.mode_valid && s.mode==1 && s.version_valid && !s.timeouts);
        assert(!strcmp(s.version,id<=3?"2.1.16STG":"0.97"));
    }
    output_len=0;output[0]=0;
    input("servo_status\r\nservo_test_001_1\n");run(780);
    const char *begin=strstr(output,"SERVO_STATUS BEGIN"), *end=strstr(output,"SERVO_STATUS END");
    const char *queued=strstr(output,"SERVO_TEST QUEUED"), *done=strstr(output,"SERVO_TEST DONE");
    assert(begin && end && end>begin && queued && done && queued<done);
    assert(strstr(begin,"mode=1 mode_valid=1 mode_check=CACHED version=2.1.16STG") &&
           strstr(begin,"mode=1 mode_valid=1 mode_check=CACHED version=0.97"));
    const char *rx_after_begin=strstr(begin,"SERVO_RX ");
    assert(rx_after_begin && rx_after_begin>end);
    const char *during_motion=strstr(queued,"SERVO_RX ");
    assert(during_motion && during_motion<done && moves==1 && positions[0]==1574);
}
static void delayed(const char *query,const char *reply,unsigned delay,unsigned window)
{
    usb.ready=true;assert(Servo_Init());
    bool requested=false,delivered=false;uint32_t tc=0;
    for(unsigned n=0;n<550;++n) {
        ++tick;
        if(!requested && tx_pending && tick==tx_due && !strcmp(tx,query)) {
            unsigned saved=reply_mask;reply_mask=0;bus_step();reply_mask=saved;
            tc=tick;requested=true;
        } else bus_step();
        if(requested && tick-tc==delay) { assert(rx_byte);feed(reply);delivered=true; }
        Servo_Process(tick);Campus_UsbProcess();
        if(delivered && tick-tc>delay+7U)break;
    }
    ServoState s=state(1);
    assert(delivered && s.timeouts==(unsigned)(delay>window));
    assert((!strcmp(query,"#001PVER!")?s.version_valid:s.mode_valid)==(delay<=window));
    char stamp[100];snprintf(stamp,sizeof(stamp),"tx=%s tc_ms=%lu dt_ms=%u lost=0",query,(unsigned long)tc,delay);
    const char *reference=strstr(output,stamp);assert(reference);
    const char *body=strstr(reference," ASCII=");
    assert(body && body<strchr(reference,'\n') && !strncmp(body+7,reply,strlen(reply)));
    printf("Injected %s delay=%u ms; window=%u ms; transaction timeouts=%lu\n",
           query,delay,window,(unsigned long)s.timeouts);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"raw"))raw();
    else if(!strcmp(argv[1],"overflow"))overflow();
    else if(!strcmp(argv[1],"reconnect"))reconnect();
    else if(!strcmp(argv[1],"burst"))burst();
    else if(!strcmp(argv[1],"commands"))commands();
    else if(!strcmp(argv[1],"pver15"))delayed("#001PVER!","#001PV0.97!",15,SERVO_REPLY_TIMEOUT_MS);
    else if(!strcmp(argv[1],"pver30"))delayed("#001PVER!","#001PV0.97!",30,SERVO_REPLY_TIMEOUT_MS);
    else if(!strcmp(argv[1],"pmod90"))delayed("#001PMOD!","#001PMOD1!",90,SERVO_MODE_REPLY_TIMEOUT_MS);
    else if(!strcmp(argv[1],"pmod110"))delayed("#001PMOD!","#001PMOD1!",110,SERVO_MODE_REPLY_TIMEOUT_MS);
    else assert(0);
    printf("PASS: servo raw RX %s\n",argv[1]);return 0;
}
