// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.AlgaeIntakeConstants.MotorID);
 
  /** Creates a new ClimbSubsystem. */
  public AlgaeIntakeSubsystem() {
    
    TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

    configs.StatorCurrentLimit = Constants.AlgaeIntakeConstants.STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimit = Constants.AlgaeIntakeConstants.CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = Constants.AlgaeIntakeConstants.ENABLE_STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimitEnable = Constants.AlgaeIntakeConstants.ENABLE_CURRENT_LIMIT;

    motorConfigs.Inverted = Constants.AlgaeIntakeConstants.MotorInverted;
    motorConfigs.NeutralMode = Constants.AlgaeIntakeConstants.MotorMode;

    talonFXConfigurator.apply(configs);
    talonFXConfigurator.apply(motorConfigs);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("ClimbPose", getDegrees());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
  }

  

  public Command run(DoubleSupplier input){
    return this.runEnd(() -> this.setSpeed(MathUtil.clamp(input.getAsDouble(), Constants.AlgaeIntakeConstants.MinSpeed, Constants.AlgaeIntakeConstants.MaxSpeed)), () -> this.setSpeed(0.0));
  }

  // public double getRawPose() {
  //   return Motor.getPosition().getValueAsDouble();
  // }

  // public double getDegrees() {
  //   return getRawPose()*360;
  // }
}
