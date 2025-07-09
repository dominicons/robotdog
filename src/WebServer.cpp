#include "WebServer.h"
#include <WiFi.h>
#include <ESP32WebServer.h>

ESP32WebServer server(80);
String command = "stop";
bool balanceEnabled = false;

void handleRoot() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{text-align:center;font-family:'Arial',sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;margin:0;padding:20px;}";
    html += ".btn{margin:10px;cursor:pointer;transition:all 0.3s ease;transform:scale(1);}";
    html += ".btn:hover{transform:scale(1.1);}";
    html += ".btn:active{transform:scale(0.95);}";
    html += ".row{margin:15px 0;}";
    html += ".toggle{cursor:pointer;transition:all 0.3s ease;}";
    html += ".toggle:hover{transform:scale(1.05);}";
    html += "</style>";
    html += "<script>let interval;";
    html += "function sendCmd(cmd){fetch('/cmd?move='+cmd);} ";
    html += "function startCmd(cmd){sendCmd(cmd);interval=setInterval(()=>sendCmd(cmd),6);} "; // Giảm từ 30ms xuống 6ms (x5 nhanh hơn)
    html += "function stopCmd(){sendCmd('stop');clearInterval(interval);} ";
    html += "function toggleBalance(){fetch('/balance?toggle=1').then(()=>location.reload());}";
    html += "</script></head><body>";
    html += "<h2>🐕 Robot Dog Control</h2>";
    html += "<div style='background:#f0f0f0;padding:20px;border-radius:15px;margin:20px;'>";
    html += "<div class='row'>";
    html += "<button class='btn' ontouchstart=\"startCmd('forward')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('forward')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\" style='background:linear-gradient(45deg,#4CAF50,#45a049);border:none;width:80px;height:80px;border-radius:50%;box-shadow:0 4px 8px rgba(0,0,0,0.2);'></button>";
    html += "<button class='btn' ontouchstart=\"startCmd('overlap')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('overlap')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\" style='background:linear-gradient(45deg,#FF9800,#F44336);border:none;width:80px;height:80px;border-radius:50%;margin-left:20px;box-shadow:0 4px 8px rgba(0,0,0,0.2);'></button>";
    html += "</div>";
    html += "<div class='row'>";
    html += "<button class='btn' ontouchstart=\"startCmd('stop')\" ontouchend=\"stopCmd()\" onmousedown=\"startCmd('stop')\" onmouseup=\"stopCmd()\" onmouseleave=\"stopCmd()\" style='background:linear-gradient(45deg,#607D8B,#37474F);border:none;width:80px;height:80px;border-radius:50%;box-shadow:0 4px 8px rgba(0,0,0,0.2);'></button>";
    html += "</div>";
    html += "<div class='row'>";
    html += "<button class='toggle' onclick='toggleBalance()' style='background:linear-gradient(45deg,#2196F3,#1976D2);color:white;border:none;padding:15px 30px;font-size:1.2em;border-radius:25px;box-shadow:0 4px 8px rgba(0,0,0,0.2);'>Balance: ";
    html += (balanceEnabled ? "ON" : "OFF");
    html += "</button>";
    html += "</div></div></body></html>";
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
    command = ""; // Reset về empty string thay vì "none"
    return tmp;
}

bool WebServerControl::isBalanceEnabled() {
    return balanceEnabled;
}
