#include "zp_servo.h"
#include <stdio.h>
#include <string.h>

size_t Zp_Build(char *out, size_t capacity, ZpOperation op, uint8_t id, uint16_t position, uint16_t time_ms)
{
    if (!out || !capacity || id == 255U) return 0;
    const char *suffix;
    switch (op) {
    case ZP_POSITION: suffix="PRAD!"; break;
    case ZP_ID: suffix="PID!"; break;
    case ZP_VERSION: suffix="PVER!"; break;
    case ZP_MODE: suffix="PMOD!"; break;
    case ZP_STOP: suffix="PDST!"; break;
    case ZP_PAUSE: suffix="PDPT!"; break;
    case ZP_RESUME: suffix="PDCT!"; break;
    case ZP_TORQUE_ON: suffix="PULR!"; break;
    case ZP_TORQUE_OFF: suffix="PULK!"; break;
    case ZP_MOVE: suffix=NULL; break;
    default: return 0;
    }
    if (op == ZP_MOVE && (position < 500U || position > 2500U || time_ms > 9999U)) return 0;
    int n = suffix ? snprintf(out,capacity,"#%03u%s",(unsigned)id,suffix) :
        snprintf(out,capacity,"#%03uP%04uT%04u!",(unsigned)id,(unsigned)position,(unsigned)time_ms);
    return n > 0 && (size_t)n < capacity ? (size_t)n : 0;
}

static bool digits(const char *p, unsigned n, unsigned *out)
{
    unsigned v=0;
    for (unsigned i=0;i<n;++i) { if(p[i]<'0'||p[i]>'9')return false; v=v*10U+(unsigned)(p[i]-'0'); }
    *out=v; return true;
}

bool Zp_Parse(const char *f, size_t n, ZpReply *out)
{
    if (!f || !out || n < 4U || n > ZP_FRAME_MAX || f[n-1U]!='!') return false;
    ZpReply r={0}; unsigned id, v;
    if (n==4U && memcmp(f,"#OK!",4)==0) { r.type=ZP_REPLY_OK; *out=r; return true; }
    if(n<6U || f[0]!='#' || f[4]!='P' || !digits(f+1,3,&id) || id>254U) return false;
    r.id=(uint8_t)id;
    if(n==6U) r.type=ZP_REPLY_ID;
    else if(n==10U && digits(f+5,4,&v) && v>=500U && v<=2500U) {
        r.type=ZP_REPLY_POSITION; r.value=(uint16_t)v;
    } else if(n==10U && memcmp(f+5,"MOD",3)==0 && f[8]>='1' && f[8]<='8') {
        r.type=ZP_REPLY_MODE; r.value=(uint16_t)(f[8]-'0');
    } else if(n>=8U && f[5]=='V' && n-7U < sizeof(r.version)) {
        r.type=ZP_REPLY_VERSION;
        for(size_t i=6;i<n-1U;++i) {
            char c=f[i];
            if(!((c>='0'&&c<='9')||(c>='A'&&c<='Z')||(c>='a'&&c<='z')||c=='.'||c=='_'||c=='-'))return false;
        }
        memcpy(r.version,f+6,n-7U);
    } else return false;
    *out=r; return true;
}

bool Zp_ExpectsReply(ZpOperation op) { return op != ZP_MOVE; }
bool Zp_Matches(ZpOperation op, uint8_t id, const ZpReply *r)
{
    if(!r)return false;
    if(op>=ZP_STOP && op<=ZP_TORQUE_OFF)return r->type==ZP_REPLY_OK;
    if(r->id!=id)return false;
    return (op==ZP_POSITION && r->type==ZP_REPLY_POSITION) || (op==ZP_ID && r->type==ZP_REPLY_ID) ||
           (op==ZP_VERSION && r->type==ZP_REPLY_VERSION) || (op==ZP_MODE && r->type==ZP_REPLY_MODE);
}
