package org.sciborgs1155.robot.pathfinding;

public class Convenience {
  // exactly as it sounds. this just makes it easier for me

  /**
   * A record that stores two integers.
   *
   * @param intOne It's an integer.
   * @param intTwo You won't believe this. It's another integer. Groundbreaking stuff huh
   */
  public record Point(int x, int y) {

    /**
     * @return x
     */
    public final int getX() {
      return x;
    }

    /**
     * @return y
     */
    public final int getY() {
      return y;
    }
  }
}
