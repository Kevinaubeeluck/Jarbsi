#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "sachinator13";
const char* password = "hahahaha";

WebServer server(80);
int want_speed = 0;
int turn = 0;
int c = 1, d = 2, e = 3;

unsigned long lastUpdate = 0;

void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Keyboard + Live Display</title>
      <style>
        body { font-family: Arial; text-align: center; margin-top: 50px; }
        h1 { font-size: 2em; }
        .value { font-size: 1.5em; color: blue; }
        textarea {
          width: 60%;
          height: 100px;
          font-size: 1em;
          margin-top: 20px;
          border-radius: 8px;
          padding: 10px;
        }
      </style>
    </head>
    <body>
      <h1>ESP32 Control Panel</h1>
      <p>want_speed = <span id="speed_val" class="value">0</span></p>
      <p>turn = <span id="turn_val" class="value">0</span></p>
      <textarea id="liveBox" readonly></textarea>

      <script>
        document.addEventListener('keydown', function(event) {
          let key = event.key.toLowerCase();
          if (key === 'w') fetch('/set?want_speed=5');
          else if (key === 's') fetch('/set?want_speed=-5');
          else if (key === 'x') fetch('/set?want_speed=0&turn=0');
          else if (key === 'a') fetch('/set?turn=5');
          else if (key === 'd') fetch('/set?turn=-5');
        });

        async function fetchValues() {
          const res = await fetch('/values');
          const data = await res.json();
          document.getElementById('speed_val').innerText = data.want_speed;
          document.getElementById('turn_val').innerText = data.turn;
          document.getElementById('liveBox').value = `c: ${data.c}\nd: ${data.d}\ne: ${data.e}`;
        }

        setInterval(fetchValues, 500);
      </script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("want_speed")) want_speed = server.arg("want_speed").toInt();
  if (server.hasArg("turn")) turn = server.arg("turn").toInt();
  server.send(200, "text/plain", "Values updated");
}

void handleValues() {
  String json = "{\"want_speed\":" + String(want_speed) +
                ",\"turn\":" + String(turn) +
                ",\"c\":" + String(c) +
                ",\"d\":" + String(d) +
                ",\"e\":" + String(e) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/values", handleValues);
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();

  // Dummy update of c, d, e every 1 second
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    c++; d++; e++;
  }
}
