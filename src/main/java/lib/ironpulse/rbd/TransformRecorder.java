package lib.ironpulse.rbd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import lombok.Getter;
import lombok.Setter;

import java.util.*;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.rbd.TransformTools.inverse;
import static lib.ironpulse.rbd.TransformTools.toTransform3d;

public class TransformRecorder {
    public static final String kFrameWorld = "World";
    public static final String kFrameRobot = "Robot";
    public static final String kFrameDriverStationBlue = "DriverStationBlue";
    public static final String kFrameDriverStationRed = "DriverStationRed";

    public static final Distance kFieldLength = Feet.of(54).plus(Inch.of(3));
    public static final Distance kFieldWidth = Feet.of(26).plus(Inch.of(3));
    public static final Pose3d kTransformWorldDriverStationBlue = new Pose3d(
            new Pose2d(0, 0, Rotation2d.kZero)
    );
    public static final Pose3d kTransformWorldDriverStationRed = new Pose3d(
            new Pose2d(kFieldLength, kFieldWidth, Rotation2d.k180deg)
    );

    private final TreeMap<String, TransformNode> frameTree;

    @Getter
    @Setter
    private double bufferDuration = 3.0;

    public TransformRecorder() {
        frameTree = new TreeMap<>();
    }

    public void putTransform(Pose3d transform, Time time, String from, String to, boolean isStatic) {
        // Ensure both frames exist in the tree
        frameTree.computeIfAbsent(from, TransformNode::new);
        frameTree.computeIfAbsent(to, TransformNode::new);

        TransformNode fromNode = frameTree.get(from);
        TransformNode toNode = frameTree.get(to);

        if (isStatic) {
            // Add static transform from -> to
            fromNode.staticChildren.put(to, new StaticTransform(transform));
            // Set parent relationship (to's parent is from)
            toNode.parent = from;
        } else {
            // Add dynamic transform from -> to
            double timeSec = time.in(Seconds);
            DynamicTransform dynTransform = fromNode.dynamicChildren.computeIfAbsent(
                    to, k -> new DynamicTransform(bufferDuration)
            );
            dynTransform.addSample(timeSec, transform);
            // Set parent relationship for dynamic transforms too
            if (toNode.parent == null) {
                toNode.parent = from;
            }
        }
    }

    public void putTransform(Pose3d transform, String from, String to) {
        putTransform(transform, Seconds.of(0.0), from, to, true);
    }

    public void putTransform(Pose3d transform, Time time, String from, String to) {
        putTransform(transform, time, from, to, false);
    }

    public void resetTransform(String from, String to) {
        frameTree.computeIfAbsent(from, TransformNode::new);
        frameTree.computeIfAbsent(to, TransformNode::new);
        TransformNode fromNode = frameTree.get(from);
        fromNode.staticChildren.remove(to);
        fromNode.dynamicChildren.remove(to);
    }

    public void resetStaticTransform(String from, String to) {
        frameTree.computeIfAbsent(from, TransformNode::new);
        frameTree.computeIfAbsent(to, TransformNode::new);
        TransformNode fromNode = frameTree.get(from);
        fromNode.staticChildren.remove(to);
    }


    public void resetDynamicTransform(String from, String to) {
        frameTree.computeIfAbsent(from, TransformNode::new);
        frameTree.computeIfAbsent(to, TransformNode::new);
        TransformNode fromNode = frameTree.get(from);
        fromNode.dynamicChildren.remove(to);
    }


    public void putTransformWorldRobot(Pose2d transform, Time time) {
        putTransform(new Pose3d(transform), time, kFrameWorld, kFrameRobot);
    }


    public Pose2d getTransformWorldRobot(Time time) {
        return getTransform(time, kFrameWorld, kFrameRobot).orElseThrow().toPose2d();
    }

    public Optional<Pose3d> getTransform(Time t, String from, String to) {
        if (to.equals(from)) {
            return Optional.of(new Pose3d());
        }

        if (!frameTree.containsKey(from) || !frameTree.containsKey(to)) {
            return Optional.empty();
        }

        double timeSec = t.in(Seconds);

        // Find path from 'from' to 'to' using tree traversal
        List<String> pathFromToRoot = getPathToRoot(from);
        List<String> pathToToRoot = getPathToRoot(to);

        // Find common ancestor
        String commonAncestor = findCommonAncestor(pathFromToRoot, pathToToRoot);
        if (commonAncestor == null) {
            return Optional.empty(); // No path exists
        }

        // Build transform: from -> commonAncestor -> to
        Optional<Pose3d> fromToCommon = getTransformToAncestor(from, commonAncestor, timeSec);
        Optional<Pose3d> commonToTo = getTransformFromAncestor(commonAncestor, to, timeSec);

        if (fromToCommon.isEmpty() || commonToTo.isEmpty()) {
            return Optional.empty();
        }

        // Combine transforms: from -> common -> to
        Pose3d result = fromToCommon.get().transformBy(toTransform3d(commonToTo.get()));
        return Optional.of(result);
    }

    private List<String> getPathToRoot(String frame) {
        List<String> path = new ArrayList<>();
        String current = frame;

        while (current != null) {
            path.add(current);
            TransformNode node = frameTree.get(current);
            current = (node != null) ? node.parent : null;
        }

        return path;
    }

    private String findCommonAncestor(List<String> path1, List<String> path2) {
        Set<String> ancestors1 = new HashSet<>(path1);

        for (String frame : path2) {
            if (ancestors1.contains(frame)) {
                return frame;
            }
        }

        return null;
    }

    private Optional<Pose3d> getTransformToAncestor(String from, String ancestor, double timeSec) {
        if (from.equals(ancestor)) {
            return Optional.of(new Pose3d());
        }

        Pose3d accumulated = new Pose3d();
        String current = from;

        while (current != null && !current.equals(ancestor)) {
            TransformNode node = frameTree.get(current);
            if (node == null || node.parent == null) {
                return Optional.empty();
            }

            // Get transform from current to its parent
            Optional<Pose3d> transform = getDirectTransform(current, node.parent, timeSec);
            if (transform.isEmpty()) {
                return Optional.empty();
            }

            accumulated = accumulated.transformBy(toTransform3d(transform.get()));
            current = node.parent;
        }

        return current != null ? Optional.of(accumulated) : Optional.empty();
    }

    private Optional<Pose3d> getTransformFromAncestor(String ancestor, String to, double timeSec) {
        if (ancestor.equals(to)) {
            return Optional.of(new Pose3d());
        }

        // Build path from ancestor to target
        List<String> pathToTarget = new ArrayList<>();
        String current = to;

        while (current != null && !current.equals(ancestor)) {
            pathToTarget.add(current);
            TransformNode node = frameTree.get(current);
            current = (node != null) ? node.parent : null;
        }

        if (current == null) {
            return Optional.empty(); // No path found
        }

        // Reverse path to go from ancestor to target
        Collections.reverse(pathToTarget);

        Pose3d accumulated = new Pose3d();
        String prev = ancestor;

        for (String frame : pathToTarget) {
            // Get transform from prev to frame (inverse of frame to prev)
            Optional<Pose3d> transform = getDirectTransform(frame, prev, timeSec);
            if (transform.isEmpty()) {
                return Optional.empty();
            }

            // Invert the transform since we're going in opposite direction
            accumulated = accumulated.transformBy(toTransform3d(inverse(transform.get())));
            prev = frame;
        }

        return Optional.of(accumulated);
    }

    private Optional<Pose3d> getDirectTransform(String from, String to, double timeSec) {
        TransformNode fromNode = frameTree.get(from);
        TransformNode toNode = frameTree.get(to);

        if (fromNode == null || toNode == null) {
            return Optional.empty();
        }

        // Check static transform from -> to
        StaticTransform staticTransform = fromNode.staticChildren.get(to);
        if (staticTransform != null) {
            return Optional.of(staticTransform.transform);
        }

        // Check dynamic transform from -> to
        DynamicTransform dynamicTransform = fromNode.dynamicChildren.get(to);
        if (dynamicTransform != null) {
            return dynamicTransform.getSample(timeSec);
        }

        // Check static transform to -> from (inverse)
        staticTransform = toNode.staticChildren.get(from);
        if (staticTransform != null) {
            return Optional.of(inverse(staticTransform.transform));
        }

        // Check dynamic transform to -> from (inverse)
        dynamicTransform = toNode.dynamicChildren.get(from);
        if (dynamicTransform != null) {
            Optional<Pose3d> pose = dynamicTransform.getSample(timeSec);
            return pose.map(TransformTools::inverse);
        }

        return Optional.empty();
    }

    // Utility methods for debugging and inspection
    public Set<String> getAllFrames() {
        return new TreeSet<>(frameTree.keySet());
    }

    public Optional<String> getParent(String frame) {
        TransformNode node = frameTree.get(frame);
        return node != null ? Optional.ofNullable(node.parent) : Optional.empty();
    }

    public Set<String> getChildren(String frame) {
        TransformNode node = frameTree.get(frame);
        if (node == null) {
            return Collections.emptySet();
        }

        Set<String> children = new TreeSet<>();
        children.addAll(node.staticChildren.keySet());
        children.addAll(node.dynamicChildren.keySet());
        return children;
    }

    private static class TransformNode {
        final String frameName;
        final TreeMap<String, StaticTransform> staticChildren;
        final TreeMap<String, DynamicTransform> dynamicChildren;
        String parent;

        TransformNode(String frameName) {
            this.frameName = frameName;
            this.staticChildren = new TreeMap<>();
            this.dynamicChildren = new TreeMap<>();
            this.parent = null;
        }
    }

    private static class StaticTransform {
        final Pose3d transform;

        StaticTransform(Pose3d transform) {
            this.transform = transform;
        }
    }

    private static class DynamicTransform {
        final TimeInterpolatableBuffer<Pose3d> buffer;

        DynamicTransform(double bufferDuration) {
            this.buffer = TimeInterpolatableBuffer.createBuffer(bufferDuration);
        }

        void addSample(double time, Pose3d pose) {
            buffer.addSample(time, pose);
        }

        Optional<Pose3d> getSample(double time) {
            return buffer.getSample(time);
        }
    }
}