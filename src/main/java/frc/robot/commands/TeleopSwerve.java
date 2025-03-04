package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.math.MathContext;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve swerveSubsystem;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier robotCentricSupplier;
    private DoubleSupplier elevHeight;
    private BooleanSupplier overrideHeight;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier elevHeight, BooleanSupplier overrideHeight) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSupplier = translationSup;
        this.strafeSupplier = strafeSup;
        this.rotationSupplier = rotationSup;
        this.robotCentricSupplier = robotCentricSup;
        this.elevHeight = elevHeight;
        this.overrideHeight = overrideHeight;
    }

    @Override
    public void execute() {
        double MinHeight = Constants.ElevatorConstants.L0Pose;
        double MaxHeight = Constants.ElevatorConstants.L4Pose;
        double Height = (MaxHeight - MinHeight) - (MathUtil.clamp(elevHeight.getAsDouble(), MinHeight, MaxHeight) - MinHeight);
        
        double multi = MathUtil.applyDeadband(Height/(MaxHeight- MinHeight), Constants.Swerve.MIN_SPEED/Constants.Swerve.MAX_SPEED, 1);
        /* Get Values, Deadband */
        if(overrideHeight.getAsBoolean()){
            multi = 1;
        }
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND) * multi;
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND) * multi;
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND) * multi;

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSupplier.getAsBoolean(),
                true);
        
        
    }
}