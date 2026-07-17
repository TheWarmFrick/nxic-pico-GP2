// Web configurator mode: the device enumerates as a USB CDC-NCM network
// adapter (like GP2040-CE's web config), runs a tiny DHCP server and serves
// a settings page at http://192.168.7.1.
//
// Entered by holding Ctrl+Alt+W in controller mode (see main.c); exited via
// the "reboot" button on the page or by replugging.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "tusb.h"

#include "lwip/etharp.h"
#include "lwip/init.h"
#include "netif/ethernet.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"

#include "config_mode.h"
#include "settings.h"

//--------------------------------------------------------------------
// Addresses
//--------------------------------------------------------------------
#define CFG_IP4(a, b, c, d) PP_HTONL(LWIP_MAKEU32(a, b, c, d))

static const ip4_addr_t our_ip = {CFG_IP4(192, 168, 7, 1)};
static const ip4_addr_t our_mask = {CFG_IP4(255, 255, 255, 0)};
static const ip4_addr_t client_ip = {CFG_IP4(192, 168, 7, 16)};

// MAC presented to the host (filled from the board id at startup)
uint8_t tud_network_mac_address[6] = {0x02, 0x4E, 0x58, 0x00, 0x00, 0x01};

//--------------------------------------------------------------------
// TinyUSB NCM <-> lwIP glue (based on tinyusb's net_lwip_webserver example)
//--------------------------------------------------------------------
static struct netif netif_data;
static struct pbuf *received_frame;

// Diagnostic ladder shown on the LED so connectivity problems can be
// located at a glance:
//   0 purple: config mode running, no traffic from the host yet
//   1 blue  : ethernet frames arriving over NCM (USB data path OK)
//   2 cyan  : DHCP lease handed out
//   3 green : HTTP request served
static int diag_stage;

static void diag_reach(int stage) {
    if (stage <= diag_stage) return;
    diag_stage = stage;
    switch (stage) {
        case 1: led_set_rgb(0, 0, 40); break;
        case 2: led_set_rgb(0, 32, 32); break;
        case 3: led_set_rgb(0, 40, 0); break;
        default: break;
    }
}

static err_t linkoutput_fn(struct netif *netif, struct pbuf *p) {
    (void)netif;
    absolute_time_t deadline = make_timeout_time_ms(100);
    for (;;) {
        if (!tud_ready()) return ERR_USE;
        if (tud_network_can_xmit(p->tot_len)) {
            tud_network_xmit(p, 0);
            return ERR_OK;
        }
        if (time_reached(deadline)) return ERR_OK; // drop rather than stall
        tud_task();
    }
}

static err_t ip4_output_fn(struct netif *netif, struct pbuf *p, const ip4_addr_t *addr) {
    return etharp_output(netif, p, addr);
}

static err_t netif_init_cb(struct netif *netif) {
    netif->mtu = 1500; // IP MTU (CFG_TUD_NET_MTU includes the ethernet header)
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, tud_network_mac_address, 6);
    netif->hwaddr[5] ^= 0x01; // must differ from the host-side MAC
    netif->name[0] = 'u';
    netif->name[1] = '0';
    netif->linkoutput = linkoutput_fn;
    netif->output = ip4_output_fn;
    return ERR_OK;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (received_frame) return false; // previous frame not consumed yet
    if (size) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
        if (!p) return false;
        memcpy(p->payload, src, size);
        received_frame = p;
    }
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    (void)arg;
    struct pbuf *p = (struct pbuf *)ref;
    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

void tud_network_init_cb(void) {
    if (received_frame) {
        pbuf_free(received_frame);
        received_frame = NULL;
    }
}

static void service_traffic(void) {
    if (received_frame) {
        diag_reach(1);
        ethernet_input(received_frame, &netif_data);
        received_frame = NULL;
        tud_network_recv_renew();
    }
}

//--------------------------------------------------------------------
// Minimal DHCP server: always leases 192.168.7.16 (single USB peer)
//--------------------------------------------------------------------
static struct udp_pcb *dhcp_pcb;

static void dhcp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                      const ip_addr_t *src_addr, u16_t src_port) {
    (void)arg;
    (void)src_addr;
    (void)src_port;

    uint8_t req[320];
    uint16_t len = (uint16_t)pbuf_copy_partial(p, req, sizeof(req), 0);
    pbuf_free(p);
    if (len < 244 || req[0] != 1) return; // BOOTREQUEST only

    // Find DHCP message type (option 53)
    uint8_t msg_type = 0;
    for (uint16_t i = 240; i + 2 < len && req[i] != 255;) {
        uint8_t opt = req[i], olen = req[i + 1];
        if (opt == 53 && olen >= 1) msg_type = req[i + 2];
        if (opt == 0) i++;
        else i = (uint16_t)(i + 2 + olen);
    }
    if (msg_type != 1 && msg_type != 3) return; // DISCOVER / REQUEST only

    uint8_t reply[320];
    memset(reply, 0, sizeof(reply));
    reply[0] = 2;                       // BOOTREPLY
    reply[1] = 1;                       // ethernet
    reply[2] = 6;                       // hwaddr len
    memcpy(&reply[4], &req[4], 4);      // xid
    memcpy(&reply[10], &req[10], 2);    // flags
    memcpy(&reply[16], &client_ip, 4);  // yiaddr
    memcpy(&reply[20], &our_ip, 4);     // siaddr
    memcpy(&reply[28], &req[28], 16);   // chaddr
    reply[236] = 99; reply[237] = 130; reply[238] = 83; reply[239] = 99; // cookie

    uint16_t i = 240;
    reply[i++] = 53; reply[i++] = 1;    // message type
    reply[i++] = (msg_type == 1) ? 2 : 5; // OFFER / ACK
    reply[i++] = 54; reply[i++] = 4;    // server id
    memcpy(&reply[i], &our_ip, 4); i += 4;
    reply[i++] = 51; reply[i++] = 4;    // lease time: 1 day
    reply[i++] = 0; reply[i++] = 1; reply[i++] = 0x51; reply[i++] = 0x80;
    reply[i++] = 1; reply[i++] = 4;     // subnet mask
    memcpy(&reply[i], &our_mask, 4); i += 4;
    reply[i++] = 3; reply[i++] = 4;     // router
    memcpy(&reply[i], &our_ip, 4); i += 4;
    reply[i++] = 6; reply[i++] = 4;     // DNS server
    memcpy(&reply[i], &our_ip, 4); i += 4;
    reply[i++] = 255;
    // BOOTP messages must be at least 300 bytes; some clients drop shorter
    // packets (zero padding after the END option is legal)
    if (i < 300) i = 300;

    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, i, PBUF_RAM);
    if (!out) return;
    memcpy(out->payload, reply, i);
    // Send on our netif explicitly -- routing a limited broadcast through
    // the default-netif lookup is what simple DHCP servers get wrong
    udp_sendto_if(pcb, out, IP_ADDR_BROADCAST, 68, &netif_data);
    pbuf_free(out);
    if (msg_type == 3) diag_reach(2); // ACK sent
}

static void dhcpd_init(void) {
    dhcp_pcb = udp_new();
    udp_bind(dhcp_pcb, IP_ANY_TYPE, 67);
    udp_bind_netif(dhcp_pcb, &netif_data);
    udp_recv(dhcp_pcb, dhcp_recv, NULL);
}

//--------------------------------------------------------------------
// Settings <-> JSON
//--------------------------------------------------------------------
static int build_config_json(char *buf, size_t cap) {
    const settings_t *s = &g_settings;
    return snprintf(buf, cap,
        "{\"yaw360\":%.0f,\"pitch360\":%.0f,\"pitchLimit\":%.1f,"
        "\"invYaw\":%u,\"invPitch\":%u,\"rapid\":%u,\"led\":%u,"
        "\"kA\":%u,\"kB\":%u,\"kX\":%u,\"kY\":%u,"
        "\"kL\":%u,\"kZL\":%u,\"kLS\":%u,"
        "\"kDU\":%u,\"kDD\":%u,\"kDL\":%u,\"kDR\":%u,"
        "\"kPlus\":%u,\"kMinus\":%u,\"kHome\":%u,\"kCap\":%u,"
        "\"kLsU\":%u,\"kLsD\":%u,\"kLsL\":%u,\"kLsR\":%u,"
        "\"kRsU\":%u,\"kRsD\":%u,\"kRsL\":%u,\"kRsR\":%u,"
        "\"kYHold\":%u,\"kRapid\":%u,\"kPitchR\":%u}",
        s->yaw_counts_per_360, s->pitch_counts_per_360, s->pitch_limit_deg,
        s->invert_yaw, s->invert_pitch, s->rapid_half_period, s->led_brightness,
        s->key_btn_a, s->key_btn_b, s->key_btn_x, s->key_btn_y,
        s->key_btn_l, s->key_btn_zl, s->key_lstick_click,
        s->key_dpad_up, s->key_dpad_down, s->key_dpad_left, s->key_dpad_right,
        s->key_plus, s->key_minus, s->key_home, s->key_capture,
        s->key_ls_up, s->key_ls_down, s->key_ls_left, s->key_ls_right,
        s->key_rs_up, s->key_rs_down, s->key_rs_left, s->key_rs_right,
        s->key_y_hold_toggle, s->key_rapid_toggle, s->key_pitch_reset);
}

static bool json_num(const char *body, const char *key, float *out) {
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    *out = strtof(p + 1, NULL);
    return true;
}

static void json_f(const char *b, const char *k, float *v, float lo, float hi) {
    float f;
    if (json_num(b, k, &f)) {
        if (f < lo) f = lo;
        if (f > hi) f = hi;
        *v = f;
    }
}

static void json_u8(const char *b, const char *k, uint8_t *v, float lo, float hi) {
    float f;
    if (json_num(b, k, &f)) {
        if (f < lo) f = lo;
        if (f > hi) f = hi;
        *v = (uint8_t)f;
    }
}

static void apply_config_json(const char *body) {
    settings_t *s = &g_settings;
    json_f(body, "yaw360", &s->yaw_counts_per_360, 500.0f, 200000.0f);
    json_f(body, "pitch360", &s->pitch_counts_per_360, 500.0f, 200000.0f);
    json_f(body, "pitchLimit", &s->pitch_limit_deg, 5.0f, 89.0f);
    json_u8(body, "invYaw", &s->invert_yaw, 0, 1);
    json_u8(body, "invPitch", &s->invert_pitch, 0, 1);
    json_u8(body, "rapid", &s->rapid_half_period, 1, 50);
    json_u8(body, "led", &s->led_brightness, 0, 255);

    json_u8(body, "kA", &s->key_btn_a, 1, 0xE7);
    json_u8(body, "kB", &s->key_btn_b, 1, 0xE7);
    json_u8(body, "kX", &s->key_btn_x, 1, 0xE7);
    json_u8(body, "kY", &s->key_btn_y, 1, 0xE7);
    json_u8(body, "kL", &s->key_btn_l, 1, 0xE7);
    json_u8(body, "kZL", &s->key_btn_zl, 1, 0xE7);
    json_u8(body, "kLS", &s->key_lstick_click, 1, 0xE7);
    json_u8(body, "kDU", &s->key_dpad_up, 1, 0xE7);
    json_u8(body, "kDD", &s->key_dpad_down, 1, 0xE7);
    json_u8(body, "kDL", &s->key_dpad_left, 1, 0xE7);
    json_u8(body, "kDR", &s->key_dpad_right, 1, 0xE7);
    json_u8(body, "kPlus", &s->key_plus, 1, 0xE7);
    json_u8(body, "kMinus", &s->key_minus, 1, 0xE7);
    json_u8(body, "kHome", &s->key_home, 1, 0xE7);
    json_u8(body, "kCap", &s->key_capture, 1, 0xE7);
    json_u8(body, "kLsU", &s->key_ls_up, 1, 0xE7);
    json_u8(body, "kLsD", &s->key_ls_down, 1, 0xE7);
    json_u8(body, "kLsL", &s->key_ls_left, 1, 0xE7);
    json_u8(body, "kLsR", &s->key_ls_right, 1, 0xE7);
    json_u8(body, "kRsU", &s->key_rs_up, 1, 0xE7);
    json_u8(body, "kRsD", &s->key_rs_down, 1, 0xE7);
    json_u8(body, "kRsL", &s->key_rs_left, 1, 0xE7);
    json_u8(body, "kRsR", &s->key_rs_right, 1, 0xE7);
    json_u8(body, "kYHold", &s->key_y_hold_toggle, 1, 0xE7);
    json_u8(body, "kRapid", &s->key_rapid_toggle, 1, 0xE7);
    json_u8(body, "kPitchR", &s->key_pitch_reset, 1, 0xE7);
}

//--------------------------------------------------------------------
// Embedded settings page
//--------------------------------------------------------------------
static const char INDEX_HTML[] =
"<!DOCTYPE html>\n"
"<html lang='ja'><head><meta charset='utf-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>NXIC-pico Web\xe8\xa8\xad\xe5\xae\x9a</title>\n"
"<style>\n"
"body{font-family:sans-serif;max-width:720px;margin:20px auto;padding:0 12px;background:#14181d;color:#e8eaed}\n"
"h1{font-size:1.4em}h2{font-size:1.1em;border-bottom:1px solid #333;padding-bottom:4px;margin-top:24px}\n"
"label{display:block;margin:8px 0 2px;font-size:.9em;color:#aaa}\n"
"input[type=number]{width:110px;background:#20262e;color:#e8eaed;border:1px solid #444;border-radius:4px;padding:6px}\n"
".keys{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}\n"
".key{background:#20262e;border:1px solid #444;border-radius:6px;padding:8px;cursor:pointer;text-align:center}\n"
".key.listen{border-color:#4af;background:#123}\n"
".key .n{font-size:.75em;color:#9ab}.key .v{font-size:1.05em;font-weight:bold}\n"
".chk{display:inline-block;margin-right:14px}\n"
"button{background:#2d6cdf;color:#fff;border:0;border-radius:6px;padding:10px 20px;margin:18px 8px 0 0;cursor:pointer;font-size:1em}\n"
"button.sec{background:#444}\n"
"#msg{margin-top:10px;color:#7fd67f}\n"
"</style></head><body>\n"
"<h1>NXIC-pico Web\xe8\xa8\xad\xe5\xae\x9a</h1>\n"
"<h2>\xe3\x82\xb8\xe3\x83\xa3\xe3\x82\xa4\xe3\x83\xad / \xe3\x82\xa8\xe3\x82\xa4\xe3\x83\xa0</h2>\n"
"<label>\xe3\x83\xa8\xe3\x83\xbc\xe6\x84\x9f\xe5\xba\xa6 (360\xc2\xb0\xe3\x81\xab\xe5\xbf\x85\xe8\xa6\x81\xe3\x81\xaa\xe3\x82\xab\xe3\x82\xa6\xe3\x83\xb3\xe3\x83\x88\xe6\x95\xb0\xe3\x80\x81\xe5\xb0\x8f\xe3\x81\x95\xe3\x81\x84\xe3\x81\xbb\xe3\x81\xa9\xe9\xab\x98\xe6\x84\x9f\xe5\xba\xa6)</label><input id='yaw360' type='number' step='100'>\n"
"<label>\xe3\x83\x94\xe3\x83\x83\xe3\x83\x81\xe6\x84\x9f\xe5\xba\xa6 (\xe5\x90\x8c\xe4\xb8\x8a)</label><input id='pitch360' type='number' step='100'>\n"
"<label>\xe3\x83\x94\xe3\x83\x83\xe3\x83\x81\xe5\x8f\xaf\xe5\x8b\x95\xe5\x9f\x9f (\xc2\xb1\xe5\xba\xa6\xe3\x80\x81" "90\xe6\x9c\xaa\xe6\xba\x80)</label><input id='pitchLimit' type='number' step='1' max='89'>\n"
"<div style='margin-top:8px'>\n"
"<label class='chk'><input id='invYaw' type='checkbox'> \xe5\xb7\xa6\xe5\x8f\xb3\xe5\x8f\x8d\xe8\xbb\xa2</label>\n"
"<label class='chk'><input id='invPitch' type='checkbox'> \xe4\xb8\x8a\xe4\xb8\x8b\xe5\x8f\x8d\xe8\xbb\xa2</label>\n"
"</div>\n"
"<h2>\xe3\x81\x9d\xe3\x81\xae\xe4\xbb\x96</h2>\n"
"<label>\xe9\x80\xa3\xe5\xb0\x84\xe5\x8d\x8a\xe5\x91\xa8\xe6\x9c\x9f (\xe3\x83\xac\xe3\x83\x9d\xe3\x83\xbc\xe3\x83\x88\xe6\x95\xb0\xe3\x80\x81" "1\xe3\x83\xac\xe3\x83\x9d\xe3\x83\xbc\xe3\x83\x88=15ms)</label><input id='rapid' type='number' min='1' max='50'>\n"
"<label>LED\xe8\xbc\x9d\xe5\xba\xa6 (0-255)</label><input id='led' type='number' min='0' max='255'>\n"
"<h2>\xe3\x82\xad\xe3\x83\xbc\xe9\x85\x8d\xe7\xbd\xae (\xe3\x82\xaf\xe3\x83\xaa\xe3\x83\x83\xe3\x82\xaf\xe3\x81\x97\xe3\x81\xa6\xe3\x82\xad\xe3\x83\xbc\xe3\x82\x92\xe6\x8a\xbc\xe3\x81\x99\xe3\x81\xa8\xe5\xa4\x89\xe6\x9b\xb4)</h2>\n"
"<div class='keys' id='keys'></div>\n"
"<button onclick='save()'>\xe4\xbf\x9d\xe5\xad\x98</button>\n"
"<button class='sec' onclick='reboot()'>\xe3\x82\xb3\xe3\x83\xb3\xe3\x83\x88\xe3\x83\xad\xe3\x83\xbc\xe3\x83\xa9\xe3\x83\xbc\xe3\x83\xa2\xe3\x83\xbc\xe3\x83\x89\xe3\x81\xa7\xe5\x86\x8d\xe8\xb5\xb7\xe5\x8b\x95</button>\n"
"<div id='msg'></div>\n"
"<script>\n"
"const KEYS=[['kA','A'],['kB','B'],['kX','X'],['kY','Y'],['kL','L'],['kZL','ZL'],['kLS','L\xe3\x82\xb9\xe3\x83\x86\xe3\x82\xa3\xe3\x83\x83\xe3\x82\xaf\xe6\x8a\xbc\xe8\xbe\xbc'],\n"
"['kDU','\xe5\x8d\x81\xe5\xad\x97 \xe4\xb8\x8a'],['kDD','\xe5\x8d\x81\xe5\xad\x97 \xe4\xb8\x8b'],['kDL','\xe5\x8d\x81\xe5\xad\x97 \xe5\xb7\xa6'],['kDR','\xe5\x8d\x81\xe5\xad\x97 \xe5\x8f\xb3'],\n"
"['kPlus','+'],['kMinus','-'],['kHome','HOME'],['kCap','\xe3\x82\xad\xe3\x83\xa3\xe3\x83\x97\xe3\x83\x81\xe3\x83\xa3'],\n"
"['kLsU','LS \xe4\xb8\x8a'],['kLsD','LS \xe4\xb8\x8b'],['kLsL','LS \xe5\xb7\xa6'],['kLsR','LS \xe5\x8f\xb3'],\n"
"['kRsU','RS \xe4\xb8\x8a'],['kRsD','RS \xe4\xb8\x8b'],['kRsL','RS \xe5\xb7\xa6'],['kRsR','RS \xe5\x8f\xb3'],\n"
"['kYHold','Y\xe3\x83\x9b\xe3\x83\xbc\xe3\x83\xab\xe3\x83\x89\xe5\x88\x87\xe6\x9b\xbf'],['kRapid','\xe9\x80\xa3\xe5\xb0\x84\xe5\x88\x87\xe6\x9b\xbf'],['kPitchR','\xe3\x83\x94\xe3\x83\x83\xe3\x83\x81\xe3\x83\xaa\xe3\x82\xbb\xe3\x83\x83\xe3\x83\x88']];\n"
"const M={};\n"
"for(let i=0;i<26;i++)M['Key'+String.fromCharCode(65+i)]=4+i;\n"
"for(let i=1;i<=9;i++)M['Digit'+i]=29+i;M['Digit0']=39;\n"
"Object.assign(M,{Enter:40,Escape:41,Backspace:42,Tab:43,Space:44,Minus:45,Equal:46,BracketLeft:47,BracketRight:48,Backslash:49,Semicolon:51,Quote:52,Backquote:53,Comma:54,Period:55,Slash:56,CapsLock:57,ArrowRight:79,ArrowLeft:80,ArrowDown:81,ArrowUp:82,Insert:73,Home:74,PageUp:75,Delete:76,End:77,PageDown:78});\n"
"for(let i=1;i<=12;i++)M['F'+i]=57+i;\n"
"const R={};for(const k in M)R[M[k]]=k;\n"
"let cfg={};let listening=null;\n"
"function keyName(v){return R[v]||('0x'+Number(v).toString(16))}\n"
"function render(){\n"
" const d=document.getElementById('keys');d.innerHTML='';\n"
" for(const [id,name] of KEYS){\n"
"  const el=document.createElement('div');el.className='key';\n"
"  el.innerHTML=`<div class='n'>${name}</div><div class='v'>${keyName(cfg[id])}</div>`;\n"
"  el.onclick=()=>listen(el,id);d.appendChild(el);\n"
" }\n"
"}\n"
"function listen(el,id){\n"
" if(listening)render();\n"
" listening={el,id};el.classList.add('listen');\n"
" el.querySelector('.v').textContent='\xe3\x82\xad\xe3\x83\xbc\xe3\x82\x92\xe6\x8a\xbc\xe3\x81\x97\xe3\x81\xa6\xe3\x81\x8f\xe3\x81\xa0\xe3\x81\x95\xe3\x81\x84';\n"
"}\n"
"window.addEventListener('keydown',e=>{\n"
" if(!listening)return;\n"
" e.preventDefault();\n"
" const u=M[e.code];\n"
" if(u)cfg[listening.id]=u;\n"
" listening=null;render();\n"
"});\n"
"async function load(){\n"
" cfg=await(await fetch('/api/config')).json();\n"
" for(const f of ['yaw360','pitch360','pitchLimit','rapid','led'])document.getElementById(f).value=cfg[f];\n"
" document.getElementById('invYaw').checked=!!cfg.invYaw;\n"
" document.getElementById('invPitch').checked=!!cfg.invPitch;\n"
" render();\n"
"}\n"
"async function save(){\n"
" for(const f of ['yaw360','pitch360','pitchLimit','rapid','led'])cfg[f]=Number(document.getElementById(f).value);\n"
" cfg.invYaw=document.getElementById('invYaw').checked?1:0;\n"
" cfg.invPitch=document.getElementById('invPitch').checked?1:0;\n"
" const r=await fetch('/api/config',{method:'POST',body:JSON.stringify(cfg)});\n"
" document.getElementById('msg').textContent=r.ok?'\xe4\xbf\x9d\xe5\xad\x98\xe3\x81\x97\xe3\x81\xbe\xe3\x81\x97\xe3\x81\x9f':'\xe4\xbf\x9d\xe5\xad\x98\xe3\x81\xab\xe5\xa4\xb1\xe6\x95\x97\xe3\x81\x97\xe3\x81\xbe\xe3\x81\x97\xe3\x81\x9f';\n"
"}\n"
"async function reboot(){\n"
" await fetch('/api/reboot',{method:'POST'});\n"
" document.getElementById('msg').textContent='\xe5\x86\x8d\xe8\xb5\xb7\xe5\x8b\x95\xe4\xb8\xad...';\n"
"}\n"
"load();\n"
"</script></body></html>\n";

//--------------------------------------------------------------------
// Tiny HTTP server (raw TCP)
//--------------------------------------------------------------------
#define HTTP_CONNS 4
#define HTTP_BUF 1536

typedef struct {
    struct tcp_pcb *pcb;
    uint16_t len;
    char buf[HTTP_BUF];
} http_conn_t;

static http_conn_t conns[HTTP_CONNS];
static absolute_time_t reboot_time;
static bool reboot_pending;

static void http_conn_free(http_conn_t *c) {
    if (c->pcb) {
        tcp_arg(c->pcb, NULL);
        tcp_recv(c->pcb, NULL);
        tcp_err(c->pcb, NULL);
        tcp_close(c->pcb);
        c->pcb = NULL;
    }
    c->len = 0;
}

static void http_send_response(http_conn_t *c, const char *status,
                               const char *content_type,
                               const char *body, size_t body_len, bool body_in_flash) {
    char hdr[192];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.1 %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %u\r\n"
                     "Cache-Control: no-store\r\n"
                     "Connection: close\r\n\r\n",
                     status, content_type, (unsigned)body_len);
    tcp_write(c->pcb, hdr, (u16_t)n, TCP_WRITE_FLAG_COPY);
    if (body_len) {
        tcp_write(c->pcb, body, (u16_t)body_len,
                  body_in_flash ? 0 : TCP_WRITE_FLAG_COPY);
    }
    tcp_output(c->pcb);
    http_conn_free(c);
}

static void http_handle_request(http_conn_t *c) {
    diag_reach(3);
    c->buf[c->len] = '\0';
    char *body = strstr(c->buf, "\r\n\r\n");
    body = body ? body + 4 : c->buf + c->len;

    if (strncmp(c->buf, "GET / ", 6) == 0 ||
        strncmp(c->buf, "GET /index", 10) == 0) {
        http_send_response(c, "200 OK", "text/html; charset=utf-8",
                           INDEX_HTML, sizeof(INDEX_HTML) - 1, true);
    } else if (strncmp(c->buf, "GET /api/config", 15) == 0) {
        char json[768];
        int n = build_config_json(json, sizeof(json));
        http_send_response(c, "200 OK", "application/json", json, (size_t)n, false);
    } else if (strncmp(c->buf, "POST /api/config", 16) == 0) {
        apply_config_json(body);
        bool ok = settings_save();
        http_send_response(c, ok ? "200 OK" : "500 Internal Server Error",
                           "application/json",
                           ok ? "{\"ok\":true}" : "{\"ok\":false}",
                           ok ? 11 : 12, false);
    } else if (strncmp(c->buf, "POST /api/reboot", 16) == 0) {
        reboot_time = make_timeout_time_ms(300);
        reboot_pending = true;
        http_send_response(c, "200 OK", "application/json", "{\"ok\":true}", 11, false);
    } else {
        http_send_response(c, "404 Not Found", "text/plain", "not found", 9, false);
    }
}

static bool http_request_complete(http_conn_t *c) {
    c->buf[c->len] = '\0';
    char *hdr_end = strstr(c->buf, "\r\n\r\n");
    if (!hdr_end) return false;
    const char *cl = strstr(c->buf, "Content-Length:");
    if (cl && cl < hdr_end) {
        int want = atoi(cl + 15);
        int have = (int)(c->len - (uint16_t)(hdr_end + 4 - c->buf));
        return have >= want;
    }
    return true;
}

static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    http_conn_t *c = (http_conn_t *)arg;
    if (!p) { // remote closed
        if (c) http_conn_free(c);
        else tcp_close(pcb);
        return ERR_OK;
    }
    if (c) {
        uint16_t room = (uint16_t)(HTTP_BUF - 1 - c->len);
        uint16_t n = (uint16_t)pbuf_copy_partial(p, c->buf + c->len,
                                                 room < p->tot_len ? room : p->tot_len, 0);
        c->len = (uint16_t)(c->len + n);
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    (void)err;

    if (c && http_request_complete(c)) http_handle_request(c);
    return ERR_OK;
}

static void http_err(void *arg, err_t err) {
    (void)err;
    http_conn_t *c = (http_conn_t *)arg;
    if (c) {
        c->pcb = NULL; // already freed by lwIP
        c->len = 0;
    }
}

static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)arg;
    (void)err;
    http_conn_t *c = NULL;
    for (int i = 0; i < HTTP_CONNS; i++) {
        if (!conns[i].pcb) {
            c = &conns[i];
            break;
        }
    }
    if (!c) {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    c->pcb = pcb;
    c->len = 0;
    tcp_arg(pcb, c);
    tcp_recv(pcb, http_recv);
    tcp_err(pcb, http_err);
    return ERR_OK;
}

static void http_init(void) {
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    tcp_bind(pcb, IP_ANY_TYPE, 80);
    pcb = tcp_listen_with_backlog(pcb, 4);
    tcp_accept(pcb, http_accept);
}

//--------------------------------------------------------------------
extern bool usb_config_mode; // usb_descriptors.c

void config_mode_main(void) {
    led_set_rgb(24, 0, 32); // purple: web config mode

    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    tud_network_mac_address[3] = id.id[5];
    tud_network_mac_address[4] = id.id[6];
    tud_network_mac_address[5] = id.id[7] & 0xFE; // keep LSB clear for the flip

    lwip_init();
    netif_add(&netif_data, &our_ip, &our_mask, &our_ip, NULL, netif_init_cb,
              ethernet_input);
    netif_data.hostname = "nxic";
    netif_set_default(&netif_data);
    netif_set_up(&netif_data);

    dhcpd_init();
    http_init();

    usb_config_mode = true;
    tud_init(BOARD_TUD_RHPORT);

    for (;;) {
        tud_task();
        service_traffic();
        sys_check_timeouts();
        if (reboot_pending && time_reached(reboot_time)) {
            watchdog_reboot(0, 0, 0);
        }
    }
}
