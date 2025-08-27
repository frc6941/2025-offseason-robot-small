package frc.robot.display;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotStateRecorder;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@Getter
public class SuperstructureVisualizerPose3d{
    private Pose3d elevator1stStage = new Pose3d();
    private Pose3d elevator2ndStage = new Pose3d();
    private Pose3d endeffector = new Pose3d();
    private Pose3d coralPose = new Pose3d(10000,0,0,new Rotation3d());


    public void updateVisuals(double elevtaorHeight){
        if(-0.03<=elevtaorHeight && elevtaorHeight<= 0.77){
            elevator1stStage = new Pose3d();
            elevator2ndStage = new Pose3d(0,0, elevtaorHeight, new Rotation3d());
            endeffector = elevator2ndStage;
        }
        else if(elevtaorHeight >0.77 && elevtaorHeight <= 0.77+0.745+0.03){
            elevator1stStage = new Pose3d(0,0,elevtaorHeight-0.77,new Rotation3d());
            elevator2ndStage = new Pose3d(0,0, elevtaorHeight, new Rotation3d());
            endeffector = elevator2ndStage;
        }
        else {
            elevator2ndStage = new Pose3d();
            endeffector = new Pose3d();
            elevator1stStage = new Pose3d();
        }
    }
    public void logCoralPose(boolean hasCoral){
        Pose3d robotPose = RobotStateRecorder.getPoseWorldRobotCurrent();
        if(hasCoral){
            coralPose = new Pose3d(robotPose.getX(),0+robotPose.getY(),0.72+ robotPose.getZ()+elevator2ndStage.getZ(),new Rotation3d(robotPose.getX(),robotPose.getRotation().getY()-40,robotPose.getRotation().getZ()));
        }
        else {
            coralPose = new Pose3d(10000, 0, 0, new Rotation3d());
        }
    }

}
