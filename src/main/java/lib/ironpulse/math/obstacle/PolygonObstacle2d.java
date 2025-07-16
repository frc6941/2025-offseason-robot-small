package lib.ironpulse.math.obstacle;

import edu.wpi.first.math.geometry.Translation2d;

public class PolygonObstacle2d implements Obstacle2d{
  private Translation2d[] cornerPoints;

  public PolygonObstacle2d(Translation2d... cornerPoints) {
    this.cornerPoints = cornerPoints;
  }

  @Override
  public boolean isInside(Translation2d point) {
    double x = point.getX();
    double y = point.getY();

    // A point is in a polygon if a line from the point to infinity crosses the
    // polygon an odd number of times
    boolean odd = false;
    for (int i = 0, j = cornerPoints.length - 1; i < cornerPoints.length; i++) {
      if ((cornerPoints[i].getY() > y) != (cornerPoints[j].getY() > y)
          && (x < (y - cornerPoints[i].getY())
          * (cornerPoints[j].getX() - cornerPoints[i].getX())
          / (cornerPoints[j].getY() - cornerPoints[i].getY())
          + cornerPoints[i].getX())) {
        odd = !odd;
      }
      j = i;
    }
    return odd;
  }
}
