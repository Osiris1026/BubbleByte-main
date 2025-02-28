// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPID extends Command {


  private PIDController ElevatorPID = new PIDController (
    // 1/22/25 PID constants will be added later
    Constants.ElevatorConstants.kP,
    Constants.ElevatorConstants.kI,
    Constants.ElevatorConstants.kD
  );

  ElevatorSubsystem e_ElevatorSubsytem;
  

  /** Creates a new ElevatorPID. */
  public ElevatorPID(ElevatorSubsystem a_Elevator, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.e_ElevatorSubsytem = a_Elevator;
    ElevatorPID.setSetpoint(setpoint);
    //ElevatorPID.setTolerance(Constants.ElevatorConstants.Tolerance);
    addRequirements(a_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = ElevatorPID.calculate(e_ElevatorSubsytem.getElevatorHeight());
    e_ElevatorSubsytem.setSpeed(value + Constants.ElevatorConstants.StallSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    e_ElevatorSubsytem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
