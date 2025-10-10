void initRadio(){
  // If this is a new node, the nodeID will return 0. Once the node is configured with an ID other than 0, this
  // bit will no longer run.
  while (!mesh.getNodeID()) {
    // Wait for the nodeID to be set via Serial
    if (Serial.available()) {
      mesh.setNodeID(Serial.read());
      Serial.print("Set NodeID: ");
      Serial.println(mesh.getNodeID());
    }
  }
  // Set the PA Level to MIN and disable LNA for testing & power supply related issues
  radio.begin();
  radio.setPALevel(RF24_PA_MIN, 0);

  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  if (!mesh.begin()) {
    if (radio.isChipConnected()) {
      do {
        // mesh.renewAddress() will return MESH_DEFAULT_ADDRESS on failure to connect
        Serial.println(F("Could not connect to network.\nConnecting to the mesh..."));
      } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
    } else {
      Serial.println(F("Radio hardware not responding."));
      while (1) {
        // hold in an infinite loop
      }
    }
  }
}