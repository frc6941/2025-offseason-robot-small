import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class SwerveTests {
    @BeforeAll
    static void setup() {
    }

    @Test
    void testSwerveSetpointGenerator() {
        var swerve = new Swerve(
                Constants.Swerve.kSimConfig,
                new ImuIOSim(),
                new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                new SwerveModuleIOSim(Constants.Swerve.kSimConfig)
        );
        swerve.runVelocity(new ChassisSpeeds(10.0, 10.0, 0.0));
        swerve.periodic();
        swerve.runVelocity(new ChassisSpeeds(10.0, 10.0, 0.0));
        swerve.periodic();
        swerve.runVelocity(new ChassisSpeeds(10.0, 10.0, 0.0));
        swerve.periodic();
    }


    @AfterEach
    void reset() {
    }
}
