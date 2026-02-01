// placeholder
#ifndef _WM_STRINGS_EN_H_
#define _WM_STRINGS_EN_H_

#define WM_HTTP_HEAD "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1,user-scalable=no\"/><title>{v}</title>"
#define WM_HTTP_STYLE "<style>body{margin:0;font-family:Arial,sans-serif;background:#f7f7f7;}h1{font-size:1.8rem;margin:0 0 .5em 0;}h2{font-size:1.2rem;margin:1.5em 0 .5em 0;}h3{font-size:1rem;margin:1em 0 .5em 0;}p{margin:.5em 0;}a{color:#007bff;text-decoration:none;}a:hover{text-decoration:underline;}form{margin:0;}input,select,button{font-size:1rem;margin:.2em 0;padding:.4em .6em;border:1px solid #ccc;border-radius:4px;}input[type=checkbox]{width:auto;}button{background:#007bff;color:#fff;border:none;cursor:pointer;}button:active{background:#0056b3;}fieldset{border:1px solid #ccc;border-radius:4px;margin:1em 0;padding:1em;}legend{font-weight:bold;}label{display:block;margin:.5em 0 .2em 0;}table{border-collapse:collapse;width:100%;margin:1em 0;}th,td{border:1px solid #ccc;padding:.5em;text-align:left;}th{background:#f0f0f0;}pre{background:#eee;padding:1em;border-radius:4px;overflow-x:auto;}#msg{color:#d00;margin:.5em 0;}</style>"
#define WM_HTTP_HEAD_END "</head><body>"
#define WM_HTTP_PORTAL "<h1>{v}</h1>"
#define WM_HTTP_PORTAL_OPTIONS "<form action=\"/wifi\" method=\"get\"><button>Configure WiFi</button></form><form action=\"/0wifi\" method=\"get\"><button>Configure WiFi (No Scan)</button></form><form action=\"/wifisave\" method=\"post\"><button>Save &amp; Connect</button></form><form action=\"/info\" method=\"get\"><button>Info</button></form><form action=\"/param\" method=\"get\"><button>Parameters</button></form><form action=\"/restart\" method=\"post\"><button>Restart</button></form>"
#define WM_HTTP_WIFI_SCAN "<h2>WiFi Networks</h2>"
#define WM_HTTP_WIFI_FORM "<form action=\"/wifisave\" method=\"post\"><label for=\"s\">SSID</label><input id=\"s\" name=\"s\" maxlength=\"32\"/><label for=\"p\">Password</label><input id=\"p\" name=\"p\" type=\"password\" maxlength=\"64\"/><button>Save</button></form>"
#define WM_HTTP_WIFI_SAVED "<div id=\"msg\">Credentials Saved<br/>Attempting to connect...</div>"
#define WM_HTTP_INFO "<h2>Device Info</h2>"
#define WM_HTTP_PARAM_HEAD "<h2>Parameters</h2>"
#define WM_HTTP_PARAM_FORM "<form action=\"/paramsave\" method=\"post\">{p}<button>Save</button></form>"
#define WM_HTTP_UPDATE "<h2>Update</h2>"
#define WM_HTTP_UPDATE_FORM "<form action=\"/update\" method=\"post\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"firmware\"/><button>Update</button></form>"
#define WM_HTTP_UPDATE_RESULT "<div id=\"msg\">Update {r}</div>"
#define WM_HTTP_RESTART "<h2>Restarting...</h2>"
#define WM_HTTP_NOTFOUND "<h2>Not found</h2>"
#define WM_HTTP_BACK "<a href=\"/\">Back</a>"
#define WM_HTTP_END "</body></html>"

#endif // _WM_STRINGS_EN_H_
// przeniesione automatycznie