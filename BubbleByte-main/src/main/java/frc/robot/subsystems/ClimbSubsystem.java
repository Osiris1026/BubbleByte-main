// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  ElevatorSubsystem e_ElevatorSubsystem;
  private TalonFX Motor = new TalonFX(Constants.ClimberConstants.MotorID);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(ElevatorSubsystem e_ElevatorSubsystem) {
    this.e_ElevatorSubsystem = e_ElevatorSubsystem;
    TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();
    HardwareLimitSwitchConfigs dumbConfigs = new HardwareLimitSwitchConfigs();

    
    
    
    limitConfigs.ForwardSoftLimitThreshold = Constants.ClimberConstants.ForwardLimit;
    limitConfigs.ForwardSoftLimitEnable = Constants.ClimberConstants.LimitEnable;
    
    limitConfigs.ReverseSoftLimitThreshold = Constants.ClimberConstants.ReverseLimit;
    limitConfigs.ReverseSoftLimitEnable = Constants.ClimberConstants.LimitEnable;
    
    currentConfigs.StatorCurrentLimit = Constants.ClimberConstants.STATOR_CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimit = Constants.ClimberConstants.CURRENT_LIMIT;
    currentConfigs.StatorCurrentLimitEnable = Constants.ClimberConstants.ENABLE_STATOR_CURRENT_LIMIT;
    currentConfigs.SupplyCurrentLimitEnable = Constants.ClimberConstants.ENABLE_CURRENT_LIMIT;
    
    motorConfigs.Inverted = Constants.ClimberConstants.MotorInverted;
    motorConfigs.NeutralMode = Constants.ClimberConstants.LiftMotorMode;

    talonFXConfigurator.apply(currentConfigs);
    talonFXConfigurator.apply(motorConfigs);
    talonFXConfigurator.apply(limitConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimbPose", getAngle());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    if(e_ElevatorSubsystem.getElevatorHeight() < Constants.ClimberConstants.ElevatorLimit){
    Motor.set(MathUtil.clamp(speed, -Constants.ClimberConstants.MaxLiftSpeed,  Constants.ClimberConstants.MaxLiftSpeed));
    }else{
      Motor.set(MathUtil.clamp(speed, 0,  Constants.ClimberConstants.MaxLiftSpeed));
    }
  }

  public Command run(DoubleSupplier input){
    return this.runEnd(() -> this.setSpeed(MathUtil.clamp(input.getAsDouble(), -Constants.ClimberConstants.MaxLiftSpeed,  Constants.ClimberConstants.MaxLiftSpeed)), () -> this.setSpeed(0));
  }

  public double getRawPose() {
    return Motor.getPosition().getValueAsDouble();
  }
  public double tickToRev(double tick){
    return tick * Constants.ClimberConstants.GearRatio;
}

  public double tickToDeg(double tick){
      return tickToRev(tick) * 360;
  }

  public double getAngle() {
    return tickToDeg(getRawPose());
  }
}
