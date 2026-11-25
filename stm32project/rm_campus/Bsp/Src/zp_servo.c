#include "zp_servo.h"
#include <stdio.h>
#include <string.h>

size_t Zp_Build(char *out, size_t capacity, ZpOperation op, uint8_t id, uint16_t position, uint16_t time_ms)
{
    if (!out || !capacity || id == 255U) return 0;
    if (op == ZP_SET_MODE) {
        if (position < 1U || position > 4U) return 0;
        int n = snprintf(out, capacity, "#%03uPMOD%u!", (unsigned)id, (unsigned)position);
        return n > 0 && (size_t)n < capacity ? (size_t)n : 0;
    }
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

/* 版本及型号标识使用有界 ASCII 字段，UART 缓冲不要求以 NUL 结尾。 */
static bool valid_token(const char *text, size_t length)
{
    if (length == 0U) return false;
    for (size_t i = 0; i < length; ++i) {
        char c = text[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
              (c >= 'a' && c <= 'z') || c == '.' || c == '_' || c == '-')) return false;
    }
    return true;
}

static bool parse_version(const char *body, size_t length, ZpReply *reply)
{
    const char *end = body + length;
    const char *version;
    if (length >= 2U && memcmp(body, "PV", 2U) == 0) {
        version = body + 2;
    } else if (length >= 2U && memcmp(body, "@ ", 2U) == 0) {
        /* 实测回包：#001@ ZL-ZServo_AD_CBM V2.1.16STG! */
        const char *model = body + 2;
        const char *space = memchr(model, ' ', length - 2U);
        if (!space || end - space < 3 || space[1] != 'V' ||
            !valid_token(model, (size_t)(space - model))) return false;
        version = space + 2;
    } else return false;

    size_t n = (size_t)(end - version);
    if (n >= sizeof(reply->version) || !valid_token(version, n)) return false;
    memcpy(reply->version, version, n);
    reply->version[n] = '\0';
    reply->type = ZP_REPLY_VERSION;
    return true;
}

bool Zp_Parse(const char *f, size_t n, ZpReply *out)
{
    if (!f || !out || n < 4U || n > ZP_FRAME_MAX + 2U) return false;
    /* 组帧器通常在 ! 处调用；直接回放完整的实测抓包时也接受其末尾 CRLF。 */
    if (f[n-2U]=='\r' && f[n-1U]=='\n') n-=2U;
    if (n < 4U || n > ZP_FRAME_MAX || f[n-1U]!='!') return false;
    ZpReply r={0}; unsigned id, v;
    if (n==4U && memcmp(f,"#OK!",4)==0) { r.type=ZP_REPLY_OK; *out=r; return true; }
    if(n<6U || f[0]!='#' || !digits(f+1,3,&id) || id>254U) return false;
    r.id=(uint8_t)id;
    const char *body = f + 4;
    size_t length = n - 5U; /* 去掉 #、三位 ID 和帧尾 !。 */
    if(length==1U && body[0]=='P') r.type=ZP_REPLY_ID;
    else if(length==5U && body[0]=='P' && digits(body+1,4,&v) && v>=500U && v<=2500U) {
        r.type=ZP_REPLY_POSITION; r.value=(uint16_t)v;
    } else if ((length == 5U && memcmp(body, "PMOD", 4U) == 0) ||
               (length == 4U && memcmp(body, "MOD", 3U) == 0)) {
        char mode = body[length - 1U];
        if (mode < '1' || mode > '8') return false;
        r.type = ZP_REPLY_MODE; r.value = (uint16_t)(mode - '0');
    } else if (!parse_version(body, length, &r)) return false;
    *out=r; return true;
}

bool Zp_ExpectsReply(ZpOperation op) { return op != ZP_MOVE; }
bool Zp_Matches(ZpOperation op, uint8_t id, const ZpReply *r)
{
    if(!r)return false;
    /* Write ACK may be #OK! or a PMOD echo. Only a separate query verifies the mode. */
    if(op==ZP_SET_MODE)return r->type==ZP_REPLY_OK ||
        (r->type==ZP_REPLY_MODE && r->id==id);
    if(op>=ZP_STOP && op<=ZP_TORQUE_OFF)return r->type==ZP_REPLY_OK;
    if(r->id!=id)return false;
    return (op==ZP_POSITION && r->type==ZP_REPLY_POSITION) || (op==ZP_ID && r->type==ZP_REPLY_ID) ||
           (op==ZP_VERSION && r->type==ZP_REPLY_VERSION) || (op==ZP_MODE && r->type==ZP_REPLY_MODE);
}
