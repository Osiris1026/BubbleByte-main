package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmSubsystem extends SubsystemBase {
    TalonFX Motor = new TalonFX(Constants.AlgaeArmConstants.MotorID);

    public AlgaeArmSubsystem(){
        TalonFXConfigurator talonFXConfigurator = Motor.getConfigurator();
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();
    
    limitConfigs.ForwardSoftLimitThreshold = Constants.AlgaeArmConstants.ForwardLimit;
    limitConfigs.ForwardSoftLimitEnable = Constants.AlgaeArmConstants.LimitEnable;
    
    limitConfigs.ReverseSoftLimitThreshold = Constants.AlgaeArmConstants.ReverseLimit;
    limitConfigs.ReverseSoftLimitEnable = Constants.AlgaeArmConstants.LimitEnable;
    
        
        configs.StatorCurrentLimit = Constants.AlgaeArmConstants.STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimit = Constants.AlgaeArmConstants.CURRENT_LIMIT;
        configs.StatorCurrentLimitEnable = Constants.AlgaeArmConstants.ENABLE_STATOR_CURRENT_LIMIT;
        configs.SupplyCurrentLimitEnable = Constants.AlgaeArmConstants.ENABLE_CURRENT_LIMIT;

        motorConfigs.Inverted = Constants.AlgaeArmConstants.MotorInverted;
        motorConfigs.NeutralMode = Constants.AlgaeArmConstants.MotorMode;
        
        talonFXConfigurator.apply(configs);
        talonFXConfigurator.apply(motorConfigs);
        talonFXConfigurator.apply(limitConfigs);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaePose", getAngle());
        // This method will be called once per scheduler run
    }

    public void setSpeed(double speed) {
        Motor.set(speed);
    }

    public Command run(double speed){
    return runEnd(()-> setSpeed(speed), ()-> setSpeed(0));
  }

    public double getRawPose() {
        return Motor.getPosition().getValueAsDouble();
    }

    public double tickToRev(double tick){
        return tick * Constants.AlgaeArmConstants.GearRatio;
    }

    public double tickToDeg(double tick){
        return tickToRev(tick) * 360;
    }

    public double getAngle() {
        return getRawPose();
    }
    
}
