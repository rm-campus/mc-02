/* 复用真实 UART7、轮询和解析器，控制 TC 和回包相对于任务取事件的顺序。 */
#define main servo_fixture_main
#include "servo_test.c"
#undef main

static bool intercept_mode, never_complete, suppress_metadata;
static unsigned completion_age, injections, seen_tx;
static unsigned metadata_tx[5][2];
static bool check_spacing, have_last;
static uint32_t last_retry_ms;

uint32_t __real_ServoUart_TakeEvents(void);
uint32_t __wrap_ServoUart_TakeEvents(void)
{
    uint32_t snapshot=__real_ServoUart_TakeEvents();
    if(intercept_mode && !never_complete && tx_pending && !strcmp(tx,"#001PMOD!") &&
       tick-(tx_due-2U)>=completion_age) {
        intercept_mode=false;
        tx_due=tick;
        bus_step(); /* 发布 TC 和完整的合法应答，然后返回旧的事件快照。 */
        ++injections;
    }
    return snapshot;
}

static void step(void)
{
    ++tick;
    unsigned saved=reply_mask;
    bool metadata=tx_pending && (!strcmp(tx+4,"PMOD!") || !strcmp(tx+4,"PVER!"));
    if(metadata) {
        unsigned id=(unsigned)(tx[3]-'0');
        if(seen_tx!=tx_count) {
            ++metadata_tx[id-1][strcmp(tx+4,"PMOD!")!=0];
            if(check_spacing && id<=4U) {
                if(have_last)assert(tick-last_retry_ms>=SERVO_METADATA_RETRY_MS);
                have_last=true;last_retry_ms=tick;
            }
        }
        if(suppress_metadata)reply_mask=0;
    }
    seen_tx=tx_count;
    drop_tc=intercept_mode && tx_pending && !strcmp(tx,"#001PMOD!");
    bus_step();
    drop_tc=false;reply_mask=saved;
    Servo_Process(tick);
    Campus_UsbProcess();
}

static void run_steps(unsigned count) { while(count--)step(); }

static void test_race(unsigned age,bool missing)
{
    intercept_mode=true;never_complete=missing;completion_age=age;
    assert(Servo_Init());run_steps(700); /* 首轮自动补读到期前核对，避免重试掩盖丢包。 */
    ServoSnapshot s;Servo_CopySnapshot(&s);
    assert(s.device[0].online && s.device[0].position==1500 && s.device[0].version_valid);
    assert(s.unexpected==0 && s.malformed==0 && mode_reads==(missing?4U:5U));
    assert(s.device[0].mode_valid==!missing && s.device[0].timeouts==(unsigned)missing);
    assert(injections==(missing?0U:1U));
}

static void test_retries(void)
{
    tick=UINT32_MAX-3500U; /* 覆盖限频计时以及位置年龄跨 tick 回绕。 */
    reply_mask=15U; suppress_metadata=true;
    assert(Servo_Init());run_steps(2200);
    unsigned before[5][2];memcpy(before,metadata_tx,sizeof(before));
    check_spacing=true;
    for(unsigned ms=0;ms<10000;++ms) {
        step();
        for(unsigned id=1;id<=4;++id) {
            ServoState s=state((uint8_t)id);
            assert(s.online && tick-s.position_ms<SERVO_POSITION_STALE_MS);
            assert(!s.mode_valid && !s.version_valid);
        }
    }
    for(unsigned i=0;i<4;++i)for(unsigned f=0;f<2;++f)assert(metadata_tx[i][f]>before[i][f]);
    assert(!memcmp(metadata_tx[4],before[4],sizeof(before[4])) && !state(5).online);
    suppress_metadata=false;run_steps(10000);
    for(unsigned id=1;id<=4;++id) {
        ServoState s=state((uint8_t)id);
        assert(s.online && s.mode_valid && s.version_valid && !strcmp(s.version,"0.97"));
    }
    memcpy(before,metadata_tx,sizeof(before));
    run_steps(2200);
    assert(!memcmp(before,metadata_tx,sizeof(before)) && !moves);
}

int main(int argc,char **argv)
{
    assert(argc==2);
    if(!strcmp(argv[1],"early"))test_race(1,false);
    else if(!strcmp(argv[1],"edge"))test_race(SERVO_TX_TIMEOUT_MS,false);
    else if(!strcmp(argv[1],"missing_tc"))test_race(0,true);
    else if(!strcmp(argv[1],"retries"))test_retries();
    else assert(0);
    printf("PASS: servo feedback %s\n",argv[1]);return 0;
}
