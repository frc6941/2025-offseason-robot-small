package frc.robot.Subsystems.ElevatorSubsystem;

import java.util.ArrayList;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.ElevatorNTParamsNT;
import static frc.robot.ElevatorNTParamsNT.MAX_EXTENSION_METERS;

import static frc.robot.Constants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.Constants.Elevator.ELEVATOR_GEAR_RATIO;
import static frc.robot.Constants.Elevator.ELEVATOR_SPOOL_DIAMETER;
import static frc.robot.Constants.Elevator.kElevatorCanID;
import static frc.robot.Constants.Elevator.motorNum;
public class ElevatorIOReal implements ElevatorIO{
    private final ArrayList<TalonFX> ElevatorMotor = new ArrayList<TalonFX>(2);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> stator;
    private final StatusSignal<Current> supply;
    private final StatusSignal<Temperature> temp;
    private double setpointMeters = 0.0;
    
        public ElevatorIOReal() {
            if (motorNum%2 != 0){
                throw new IllegalArgumentException("Motor number must be even");
            }
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
            currentLimitsConfigs.StatorCurrentLimitEnable = true;
            currentLimitsConfigs.SupplyCurrentLimitEnable = true;
            currentLimitsConfigs.StatorCurrentLimit = 40.0;
            currentLimitsConfigs.SupplyCurrentLimit = 40.0;
    
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = ElevatorNTParamsNT.ElevatorGearing.kP.getValue();
            slot0Configs.kI = ElevatorNTParamsNT.ElevatorGearing.kI.getValue();
            slot0Configs.kD = ElevatorNTParamsNT.ElevatorGearing.kD.getValue();
            slot0Configs.kA = ElevatorNTParamsNT.ElevatorGearing.kA.getValue();
            slot0Configs.kV = ElevatorNTParamsNT.ElevatorGearing.kV.getValue();
            slot0Configs.kS = ElevatorNTParamsNT.ElevatorGearing.kS.getValue();
            slot0Configs.kG = ElevatorNTParamsNT.ElevatorGearing.kG.getValue();
    
            ArrayList<MotorOutputConfigs> motorOutputConfigs = new ArrayList<MotorOutputConfigs>(motorNum);
            for(int i = 0; i < motorNum; i++) {
                motorOutputConfigs.set(i, new MotorOutputConfigs());
                ElevatorMotor.set(i, new TalonFX(kElevatorCanID[i].get("MotorNum"+Integer.toString(i)),CANIVORE_CAN_BUS_NAME));
                ElevatorMotor.get(i).getConfigurator().apply(currentLimitsConfigs);
                motorOutputConfigs.get(i).NeutralMode = NeutralModeValue.Brake;
                motorOutputConfigs.get(i).Inverted = i %2 == 0 ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
                ElevatorMotor.get(i).clearStickyFaults();
                ElevatorMotor.get(i).getConfigurator().apply(slot0Configs);
                if(i%2 != 0){
                    ElevatorMotor.get(i).setControl(new Follower(i-1, true));
                }
                
            }
            velocity = ElevatorMotor.get(0).getVelocity();
            position = ElevatorMotor.get(0).getPosition();
            voltage = ElevatorMotor.get(0).getSupplyVoltage();
            stator = ElevatorMotor.get(0).getStatorCurrent();
            supply = ElevatorMotor.get(0).getSupplyCurrent();
            temp = ElevatorMotor.get(0).getDeviceTemp();
            
    
        }
        
        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            BaseStatusSignal.refreshAll(
                    velocity,
                    position,
                    voltage,
                    stator,
                    supply,
                    temp
            );
    
            inputs.positionMeters = talonPosToHeight(ElevatorMotor.get(0).getPosition().getValueAsDouble());
            inputs.velocityMetersPerSec = velocity.getValueAsDouble();
            inputs.appliedVolts = voltage.getValueAsDouble();
            inputs.statorCurrentAmps = stator.getValueAsDouble();
            inputs.supplyCurrentAmps = supply.getValueAsDouble();
            inputs.tempCelsius = temp.getValueAsDouble();
            inputs.motorVoltage = ElevatorMotor.get(0).getMotorVoltage().getValueAsDouble();
            inputs.setpointMeters = setpointMeters;
        }
    
        @Override
        public void configureSlot0Config(double kp, double ki, double kd, double ka, double kv, double ks, double kg) {
            for (int i = 0; i < motorNum; i++) {
                ElevatorMotor.get(i).getConfigurator().apply(new Slot0Configs()
                                                      .withKP(kp)
                                                      .withKI(ki)
                                                      .withKD(kd)
                                                      .withKA(ka)
                                                      .withKV(kv)
                                                      .withKS(ks)
                                                      .withKG(kg));
    
            }
        }
    
        @Override
        public void setElevatorVoltage(double volts) {
            for (int i = 0; i < motorNum; i++) {
                if (i % 2 == 0) {
                    ElevatorMotor.get(i).setControl(new VoltageOut(volts));
                }
            }
            
        }
    
        @Override
        public void setElevatorTarget(double meters) {
        setpointMeters = meters;
        for (int i = 0; i < motorNum; i++) {
            if (i % 2 == 0) {
                ElevatorMotor.get(i).setControl(new PositionDutyCycle(heightToTalonPos(Math.min(meters, MAX_EXTENSION_METERS.getValue()))));
            }
        }
        // ElevatorMotor.get(0).setControl(motionRequest.withPosition(heightToTalonPos(Math.min(meters, MAX_EXTENSION_METERS.get()))));
    }

    @Override
    public void resetElevatorPosition() {
        for (int i = 0; i < motorNum; i++) {
            ElevatorMotor.get(i).setPosition(0);
        }
    }
        

    @Override
    public double getElevatorVelocity() {
        return talonPosToHeight(ElevatorMotor.get(0).getVelocity().getValueAsDouble());
    }

    private double talonPosToHeight(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }

    private double heightToTalonPos(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }
}

