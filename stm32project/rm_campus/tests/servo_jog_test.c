/* 复用真实 USB、UART7 协议和五台模拟舵机。每个场景从全新进程开始。 */
#define main full_servo_fixture_main
#include "servo_test.c"
#undef main

static void start_test(void)
{
    usb.ready=true;
    assert(Servo_Init());
    run(1400);
    assert(moves==0 && output_len==0);
}

static void clear_log(void) { output_len=0; output[0]=0; }

static void motions(void)
{
    reply_mask=15U;
    modes[1]=2; modes[2]=3; modes[3]=4;
    start_test();
    input("servo_te"); input("st_001_1\r\nservo_test_001_1\nservo_status\n");
    run(200);
    assert(strstr(output,"SERVO_TEST QUEUED id=001") && !strstr(output,"SERVO_TEST DONE"));
    run(1350);
    assert(moves==2 && positions[0]==1648 && strstr(output,"SERVO_STATUS END"));
    assert(strstr(output,"start=1500 target=1574 actual=1574"));
    assert(strstr(output,"start=1574 target=1648 actual=1648"));
    input("servo_test_002_1\nservo_test_003_2\nservo_test_004_2\n");run(2300);
    assert(positions[1]==1426 && positions[2]==1389 && positions[3]==1611 && moves==5);
    input("servo_test_002_2\nservo_test_003_1\nservo_test_004_1\n");run(2300);
    assert(positions[1]==1500 && positions[2]==1500 && positions[3]==1500 && moves==8);
    input("servo_test_005_2\nservo_test_001_1x\nservo_test_1_1\nservo_test_000_1\n");
    run(160);assert(strstr(output,"reason=OFFLINE") && strstr(output,"reason=INVALID") && moves==8);
    clear_log();input("servo_move 001 1200 50\nservo_set_id 001 002\nservo_position_mode 1\n");
    run(120);assert(!output_len && moves==8);
}

static void modes_test(void)
{
    mode_reply_mask=30U; modes[1]=5;
    start_test();
    assert(!state(1).mode_valid);
    input("servo_test_001_1\n");run(1400);
    assert(!moves && strstr(output,"reason=MODE_UNKNOWN") && !mode_writes[0]);
    mode_reply_mask=31U;slow_mode_reply_mask=1U;
    input("servo_test_001_1\n");run(1700);
    assert(positions[0]==1574 && moves==1 && !mode_writes[0]);
    input("servo_test_002_2\n");run(300);assert(moves==1 && strstr(output,"reason=BAD_MODE"));
    positions[2]=2470;input("servo_test_003_1\n");run(200);
    assert(strstr(output,"reason=LIMIT") && moves==1 && positions[2]==2470);
    input("servo_test_005_2\n");run(900);assert(positions[4]==1426 && moves==2);
}

static void faults(void)
{
    start_test();
    input("servo_test_001_1\n");
    /* 模拟受阻：线上运动只发一次，每次 PRAD 都返回原来的实测位置。 */
    for(unsigned i=0;i<1900;++i) {
        ++tick;bus_step();positions[0]=1500;Servo_Process(tick);Campus_UsbProcess();
    }
    assert(moves==1 && strstr(output,"reason=NOT_REACHED") && !strstr(output,"SERVO_TEST DONE"));
    drop_move_tc=true;
    input("servo_test_001_2\n");run(200);drop_move_tc=false;run(850);
    assert(strstr(output,"reason=TX_TIMEOUT") && moves==1);
}

static void session_test(void)
{
    start_test();
    input("servo_test_001_1\nservo_test_001_1\n");run(120);
    assert(moves==1);
    ++usb.session_id;run(1100);assert(moves==1);
    input("servo_test_001_2\n");run(800);assert(moves==2 && positions[0]==1500);
}

static void queue_test(void)
{
    start_test();
    for(unsigned i=0;i<9;++i) { input("servo_test_001_1\n");run(10); }
    run(6500);
    assert(moves==8 && positions[0]==1500+8*74 && strstr(output,"reason=BUSY"));
}

int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"motions"))motions();
    else if(!strcmp(argv[1],"modes"))modes_test();
    else if(!strcmp(argv[1],"faults"))faults();
    else if(!strcmp(argv[1],"session"))session_test();
    else if(!strcmp(argv[1],"queue"))queue_test();
    else assert(0);
    printf("PASS: USB servo jog %s\n",argv[1]);
    return 0;
}
