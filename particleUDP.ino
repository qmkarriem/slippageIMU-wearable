// EXAMPLE USAGE

// UDP Port used for two way communication
unsigned int localPort = 8888;

// An UDP instance to let us send and receive packets over UDP
UDP Udp;

void setup() {
  // start the UDP
  Udp.begin(localPort);

  // Print your device IP Address via serial
  Serial.begin(9600);
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if data has been received
  if (Udp.parsePacket() > 0) {

    // Read first char of data received
    char c = Udp.read();

    // Ignore other chars
    while(Udp.available())
      Udp.read();

    // Store sender ip and port
    IPAddress ipAddress = Udp.remoteIP();
    int port = Udp.remotePort();

    // Echo back data to sender
    Udp.beginPacket(ipAddress, 8888);
    Udp.write(c);
    Serial.print(ipAddress); Serial.print(", "); Serial.print(port); Serial.print(", "); Serial.println(c);
    Udp.endPacket();
  }

}
