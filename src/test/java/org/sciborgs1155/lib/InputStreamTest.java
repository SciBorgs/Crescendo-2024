package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class InputStreamTest {

  private double two() {
    return 2.0;
  }

  @Test
  void signedPow() {
    var stream = InputStream.of(this::two).negate().clamp(1.8).signedPow(2);
    System.out.println(stream.get());
    assert stream.get() == -3.24;
  }

  @Test
  void deadband() {
    var stream = InputStream.of(this::two).negate().scale(0.25);
    assertEquals(stream.deadband(0.6, 1).get(), 0.0);
    assertEquals(stream.deadband(0.1, 1).get(), -0.444444, 0.01);
  }

  @Test
  void rateLimit() {
    InputStream cursed =
        new InputStream() {
          double state = 0;

          @Override
          public double getAsDouble() {
            return state += 2;
          }
        }.rateLimit(1);

    assertEquals(cursed.get(), 0, 0.1); // 0 time passes and we can't mock time
  }
}
