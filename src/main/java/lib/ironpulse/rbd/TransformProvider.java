package lib.ironpulse.rbd;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;

public interface TransformProvider {
    Transform3d getTransform(Time time);
}
