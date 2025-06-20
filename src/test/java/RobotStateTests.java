import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import lib.ironpulse.rbd.TransformRecorder;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.lang.reflect.Field;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

public class RobotStateTests {
    private TransformRecorder state;

    @BeforeEach
    void setUp() throws Exception {
        // Reset singleton for isolated tests
        Field instanceField = TransformRecorder.class.getDeclaredField("instance");
        instanceField.setAccessible(true);
        instanceField.set(null, null);
        state = new TransformRecorder();
        state.setBufferDuration(10.0);
    }

    @Test
    void testComplexKinematicChain() {
        // Create a complex robot chain: World → Robot → Turret → Camera → Sensor
        state.putTransform(new Pose3d(0, 0, 0.5, new Rotation3d()), "World", "Robot");
        state.putTransform(new Pose3d(0.2, 0, 0.1, new Rotation3d(0, 0, Math.PI / 4)), "Robot", "Turret");

        // Dynamic turret rotation
        state.putTransform(new Pose3d(0.3, 0, 0.05, new Rotation3d(0, 0, 0)), Seconds.of(0.0), "Turret", "Camera");
        state.putTransform(
                new Pose3d(0.3, 0, 0.05, new Rotation3d(0, 0, Math.PI / 2)), Seconds.of(2.0), "Turret", "Camera");

        state.putTransform(new Pose3d(0.1, 0, 0, new Rotation3d()), "Camera", "Sensor");

        // Test full chain transformation at different times
        Pose3d worldToSensor0 = state.getTransform(Seconds.of(0.0), "World", "Sensor").orElseThrow();
        Pose3d worldToSensor1 = state.getTransform(Seconds.of(1.0), "World", "Sensor").orElseThrow();
        Pose3d worldToSensor2 = state.getTransform(Seconds.of(2.0), "World", "Sensor").orElseThrow();

        // Verify positions change due to dynamic transform
        assertNotEquals(worldToSensor0.getX(), worldToSensor2.getX(), 1e-6);

        // Test reverse transformation
        Pose3d sensorToWorld = state.getTransform(Seconds.of(1.0), "Sensor", "World").orElseThrow();
        Pose3d identity = worldToSensor1.transformBy(sensorToWorld.minus(new Pose3d()));
        assertEquals(0.0, identity.getX(), 1e-6);
        assertEquals(0.0, identity.getY(), 1e-6);
        assertEquals(0.0, identity.getZ(), 1e-6);
    }

    @Test
    void testMultipleBranches() {
        // Create tree with multiple branches:
        //     World
        //    /     \
        //  RobotL  RobotR
        //  /   \   /   \
        // ArmL CamL ArmR CamR

        state.putTransform(new Pose3d(-1, 0, 0, new Rotation3d()), "World", "RobotL");
        state.putTransform(new Pose3d(1, 0, 0, new Rotation3d()), "World", "RobotR");

        state.putTransform(new Pose3d(0, -0.5, 0, new Rotation3d()), "RobotL", "ArmL");
        state.putTransform(new Pose3d(0, 0.5, 0, new Rotation3d()), "RobotL", "CamL");

        state.putTransform(new Pose3d(0, -0.5, 0, new Rotation3d()), "RobotR", "ArmR");
        state.putTransform(new Pose3d(0, 0.5, 0, new Rotation3d()), "RobotR", "CamR");

        // Test cross-branch transformations
        Pose3d armLToArmR = state.getTransform(Seconds.of(0.0), "ArmL", "ArmR").orElseThrow();
        assertEquals(2.0, armLToArmR.getX(), 1e-9); // Should be 2 units apart in X
        assertEquals(0.0, armLToArmR.getY(), 1e-9); // Same Y position

        Pose3d camLToCamR = state.getTransform(Seconds.of(0.0), "CamL", "CamR").orElseThrow();
        assertEquals(2.0, camLToCamR.getX(), 1e-9);
        assertEquals(0.0, camLToCamR.getY(), 1e-9);

        // Test diagonal transformations
        Pose3d armLToCamR = state.getTransform(Seconds.of(0.0), "ArmL", "CamR").orElseThrow();
        assertEquals(2.0, armLToCamR.getX(), 1e-9);
        assertEquals(1.0, armLToCamR.getY(), 1e-9);
    }

    @Test
    void testDynamicInterpolationWithRotations() {
        // Test complex rotation interpolation
        Rotation3d rot0 = new Rotation3d(0, 0, 0);
        Rotation3d rot2 = new Rotation3d(0, 0, Math.PI);

        state.putTransform(new Pose3d(0, 0, 0, rot0), Seconds.of(0.0), "Base", "Rotating");
        state.putTransform(new Pose3d(0, 0, 0, rot2), Seconds.of(2.0), "Base", "Rotating");

        // Test interpolation at quarter points
        Pose3d quarter = state.getTransform(Seconds.of(0.5), "Base", "Rotating").orElseThrow();
        Pose3d half = state.getTransform(Seconds.of(1.0), "Base", "Rotating").orElseThrow();
        Pose3d threeQuarter = state.getTransform(Seconds.of(1.5), "Base", "Rotating").orElseThrow();

        // Verify rotation interpolation (approximate due to slerp)
        double quarterZ = quarter.getRotation().getZ();
        double halfZ = half.getRotation().getZ();
        double threeQuarterZ = threeQuarter.getRotation().getZ();

        assertTrue(quarterZ > 0 && quarterZ < halfZ, "Quarter rotation should be between 0 and half");
        assertTrue(halfZ > quarterZ && halfZ < threeQuarterZ, "Half rotation progression");
        assertTrue(Math.abs(threeQuarterZ - Math.PI) < Math.abs(halfZ - Math.PI), "Three quarter closer to PI");
    }

    @Test
    void testBufferOverflow() {
        state.setBufferDuration(1.0); // Short buffer

        // Add many samples spanning longer than buffer duration
        for (int i = 0; i <= 20; i++) {
            double time = i * 0.1; // 0.0 to 2.0 seconds
            state.putTransform(new Pose3d(time, 0, 0, new Rotation3d()), Seconds.of(time), "A", "B");
        }

        // Old samples should be unavailable
        Optional<Pose3d> oldSample = state.getTransform(Seconds.of(0.0), "A", "B");
        assertFalse(oldSample.isEmpty(), "Old sample beyond buffer duration should be the last one");
        assertEquals(oldSample, state.getTransform(Seconds.of(1.1), "A", "B"));

        // Recent samples should be available
        Optional<Pose3d> recentSample = state.getTransform(Seconds.of(1.5), "A", "B");
        assertTrue(recentSample.isPresent(), "Recent sample within buffer should be available");
        assertEquals(1.5, recentSample.get().getX(), 1e-6);
    }

    @Test
    void testUtilityMethods() {
        Set<String> allFramesBegin = state.getAllFrames();
        // Build a small tree
        state.putTransform(new Pose3d(1, 0, 0, new Rotation3d()), "Root", "Child1");
        state.putTransform(new Pose3d(0, 1, 0, new Rotation3d()), "Root", "Child2");
        state.putTransform(new Pose3d(0, 0, 1, new Rotation3d()), "Child1", "Grandchild");

        // Test getAllFrames
        Set<String> allFrames = state.getAllFrames();
        assertEquals(4, allFrames.size() - allFramesBegin.size());
        assertTrue(allFrames.contains("Root"));
        assertTrue(allFrames.contains("Child1"));
        assertTrue(allFrames.contains("Child2"));
        assertTrue(allFrames.contains("Grandchild"));

        // Test getParent
        assertEquals("Root", state.getParent("Child1").orElse(""));
        assertEquals("Root", state.getParent("Child2").orElse(""));
        assertEquals("Child1", state.getParent("Grandchild").orElse(""));
        assertTrue(state.getParent("Root").isEmpty());

        // Test getChildren
        Set<String> rootChildren = state.getChildren("Root");
        assertEquals(2, rootChildren.size());
        assertTrue(rootChildren.contains("Child1"));
        assertTrue(rootChildren.contains("Child2"));

        Set<String> child1Children = state.getChildren("Child1");
        assertEquals(1, child1Children.size());
        assertTrue(child1Children.contains("Grandchild"));

        assertTrue(state.getChildren("Grandchild").isEmpty());
    }

    @Test
    void testMixedStaticDynamicChain() {
        state.putTransform(new Pose3d(1, 20, 0, new Rotation3d()), "A", "B");
        state.putTransform(new Pose3d(0, 103, 5, new Rotation3d(0, 0, 0)), Seconds.of(0.0), "B", "C");
        state.putTransform(new Pose3d(112, 40, 10, new Rotation3d(0, 0, Math.PI / 2)), Seconds.of(2.0), "B", "C");

        state.putTransform(new Pose3d(0, 11, 3, new Rotation3d()), "C", "D");

        state.putTransform(new Pose3d(10, 101, 1, new Rotation3d()), Seconds.of(0.0), "D", "E");
        state.putTransform(new Pose3d(21, 203, 2, new Rotation3d()), Seconds.of(2.0), "D", "E");

        // Test full chain at different times
        Pose3d ae0 = state.getTransform(Seconds.of(0.0), "A", "E").orElseThrow();
        Pose3d ae1 = state.getTransform(Seconds.of(2.0), "A", "E").orElseThrow();

        assertNotEquals(ae0.getX(), ae1.getX(), 1e-6);
        assertNotEquals(ae0.getY(), ae1.getY(), 1e-6);
        assertNotEquals(ae0.getZ(), ae1.getZ(), 1e-6);
    }

    @Test
    void testDeepChain() {
        // Create a deep linear chain: A->B->C->D->E->F->G->H->I->J
        String[] frames = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};

        for (int i = 0; i < frames.length - 1; i++) {
            state.putTransform(new Pose3d(1, 0, 0, new Rotation3d()), frames[i], frames[i + 1]);
        }

        // Test transformation across entire chain
        Pose3d aToJ = state.getTransform(Seconds.of(0.0), "A", "J").orElseThrow();
        assertEquals(9.0, aToJ.getX(), 1e-9); // 9 steps of 1 unit each

        // Test reverse
        Pose3d jToA = state.getTransform(Seconds.of(0.0), "J", "A").orElseThrow();
        assertEquals(-9.0, jToA.getX(), 1e-9);

        // Test intermediate transformations
        Pose3d cToG = state.getTransform(Seconds.of(0.0), "C", "G").orElseThrow();
        assertEquals(4.0, cToG.getX(), 1e-9); // 4 steps from C to G
    }

    @Test
    @Timeout(5)
    void testConcurrentAccess() throws InterruptedException {
        // Setup initial transform
        state.putTransform(new Pose3d(1, 0, 0, new Rotation3d()), Seconds.of(0.0), "Base", "Moving");

        ExecutorService executor = Executors.newFixedThreadPool(10);
        CountDownLatch latch = new CountDownLatch(100);

        // Concurrent writers and readers
        for (int i = 0; i < 50; i++) {
            final int index = i;
            executor.submit(() -> {
                try {
                    // Writer thread
                    state.putTransform(
                            new Pose3d(index * 0.1, 0, 0, new Rotation3d()), Seconds.of(index * 0.1), "Base", "Moving");
                } finally {
                    latch.countDown();
                }
            });

            executor.submit(() -> {
                try {
                    // Reader thread
                    Optional<Pose3d> result = state.getTransform(Seconds.of(index * 0.1), "Base", "Moving");
                    // Just verify we get some result (may be interpolated)
                    assertTrue(result.isPresent() || index == 0); // First might not be present yet
                } finally {
                    latch.countDown();
                }
            });
        }

        assertTrue(latch.await(5, TimeUnit.SECONDS), "Concurrent operations should complete");
        executor.shutdown();
    }

    @Test
    void testLargeTreePerformance() {
        // Create a wide tree with many branches
        String root = "Root";

        // Create 100 direct children of root
        for (int i = 0; i < 100; i++) {
            String child = "Child" + i;
            state.putTransform(new Pose3d(i, 0, 0, new Rotation3d()), root, child);

            // Each child has 5 grandchildren
            for (int j = 0; j < 5; j++) {
                String grandchild = child + "_" + j;
                state.putTransform(new Pose3d(0, j, 0, new Rotation3d()), child, grandchild);
            }
        }

        long startTime = System.nanoTime();

        // Test transformations between distant nodes
        for (int i = 0; i < 10; i++) {
            String from = "Child" + i + "_0";
            String to = "Child" + (99 - i) + "_4";
            Optional<Pose3d> transform = state.getTransform(Seconds.of(0.0), from, to);
            assertTrue(transform.isPresent(), "Transform should exist in large tree");
        }

        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000; // Convert to milliseconds

        // Should complete reasonably quickly (adjust threshold as needed)
        assertTrue(duration < 100, "Large tree operations should be efficient: " + duration + "ms");
    }

    @Test
    void testFrameUpdates() {
        // Test updating existing transforms
        state.putTransform(new Pose3d(1, 0, 0, new Rotation3d()), "A", "B");

        Pose3d original = state.getTransform(Seconds.of(0.0), "A", "B").orElseThrow();
        assertEquals(1.0, original.getX(), 1e-9);

        // Update the same transform (should replace for static)
        state.putTransform(new Pose3d(2, 0, 0, new Rotation3d()), "A", "B");

        Pose3d updated = state.getTransform(Seconds.of(0.0), "A", "B").orElseThrow();
        assertEquals(2.0, updated.getX(), 1e-9);
    }
}