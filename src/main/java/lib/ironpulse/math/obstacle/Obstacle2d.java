package lib.ironpulse.math.obstacle;

import edu.wpi.first.math.geometry.Translation2d;

public interface Obstacle2d {
  boolean isInside(Translation2d point);
}
