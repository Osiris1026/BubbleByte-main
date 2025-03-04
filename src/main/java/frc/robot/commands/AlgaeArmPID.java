// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmPID extends Command {
  
  private AlgaeArmSubsystem a_AlgaeArmSubsystem;
  private PIDController pid = new PIDController(
      Constants.AlgaeArmConstants.kP, 
      Constants.AlgaeArmConstants.kI, 
      Constants.AlgaeArmConstants.kD);
  /** Creates a new ClimbPID. */
  public AlgaeArmPID(AlgaeArmSubsystem subsytem, double Setpoint) {
    this.a_AlgaeArmSubsystem = subsytem;
    this.pid.setSetpoint(Setpoint);
    this.pid.setTolerance(Constants.AlgaeArmConstants.Tolerance );
    addRequirements(a_AlgaeArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pose = a_AlgaeArmSubsystem.getAngle();
    double calculated = pid.calculate(pose);
    a_AlgaeArmSubsystem.setSpeed(calculated);
    SmartDashboard.putNumber("ClimbPID", calculated);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_AlgaeArmSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
