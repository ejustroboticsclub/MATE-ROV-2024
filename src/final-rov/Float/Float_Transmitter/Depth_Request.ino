float Request_Depth() {
  Wire.requestFrom(2, 4);  // request 4 bytes from slave device #2
  if (Wire.available() >= 4) {
    union Depth_tag {
      byte Depth_b[4];
      float Depth_fval;
    } Depth_Union;
    for (int i = 0; i < 4; i++) {
      Depth_Union.Depth_b[i] = Wire.read();
    }
    Depth = Depth_Union.Depth_fval;  // Convert to Depth
  }
  return Depth;
}
