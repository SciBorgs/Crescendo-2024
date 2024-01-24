package org.sciborgs1155.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class NoLed implements LedIO {

  //  this means i don't have any LEDs
  //  this shouldn't be used because
  //
  //  Quote:
  //  "The only logical conclusion is that it is imperative for us to have working LEDs this year."
  //     -Siggy 2024, ChiefDelphi
  //
  //  https://www.chiefdelphi.com/t/frc-1155-the-sciborgs-2024-build-thread-open-alliance/441531/24

  @Override
  public void setData(AddressableLEDBuffer ledBuffer) {}

  @Override
  public void close() throws Exception {}
}
