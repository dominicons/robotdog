#include "WebServer.h"
#include <WiFi.h>
#include <ESP32WebServer.h>

ESP32WebServer server(80);
String command = "stop";
bool balanceEnabled = false;

void handleRoot() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{text-align:center;font-family:sans-serif;}";
    html += ".btn{width:60px;height:60px;font-size:2em;margin:5px;}";
    html += ".row{margin:10px;}";
    html += ".toggle{background:#4CAF50;color:white;border:none;padding:10px 20px;font-size:1em;border-radius:8px;}";
    html += "</style>";
    html += "<script>let interval;";
    html += "function sendCmd(cmd){fetch('/cmd?move='+cmd);} ";
    html += "function startCmd(cmd){sendCmd(cmd);interval=setInterval(()=>sendCmd(cmd),200);} ";
    html += "function stopCmd(){sendCmd('stop');clearInterval(interval);} ";
    html += "function toggleBalance(){fetch('/balance?toggle=1').then(()=>location.reload());}";
    html += "</script></head><body>";
    html += "<h2>Robot Control</h2>";
    html += "<div class='row'>";
    html += "<button class='btn' ontouchstart=\"startCmd('forward')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('forward')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\">&#8593;</button>";
    html += "</div>";
    html += "<div class='row'>";
    html += "<button class='btn' ontouchstart=\"startCmd('left')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('left')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\">&#8592;</button>";
    html += "<button class='btn' ontouchstart=\"startCmd('stop')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('stop')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\">&#9632;</button>";
    html += "<button class='btn' ontouchstart=\"startCmd('right')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('right')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\">&#8594;</button>";
    html += "</div>";
    html += "<div class='row'>";
    html += "<button class='btn' ontouchstart=\"startCmd('backward')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('backward')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\">&#8595;</button>";
    html += "</div>";
    html += "<div class='row'>";
    html += "<button class='toggle' onclick='toggleBalance()'>Balance: ";
    html += (balanceEnabled ? "ON" : "OFF");
    html += "</button></div></body></html>";
    server.send(200, "text/html", html);
}

void handleCmd() {
    if (server.hasArg("move")) {
        command = server.arg("move");
    }
    server.send(200, "text/plain", "OK");
}

void handleBalance() {
    balanceEnabled = !balanceEnabled;
    server.send(200, "text/plain", "OK");
}

void WebServerControl::begin() {
    WiFi.begin("URLAB", "skyholic2025");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.on("/balance", handleBalance);
    server.begin();
}

void WebServerControl::handleClient() {
    server.handleClient();
}

String WebServerControl::getCommand() {
    String tmp = command;
    command = "none";
    return tmp;
}

bool WebServerControl::isBalanceEnabled() {
    return balanceEnabled;
}
