package frc.robot.utils.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CanBus;

// import com.ctre.phoenix.ErrorCode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.*;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.signals.*;
import com.ctre.phoenixpro.spns.*;
import com.ctre.phoenixpro.hardware.core.*;
import com.ctre.phoenixpro.configs.jni.*;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.wpiutils.*;


public class SwerveModule {
    public int moduleNumber;

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANcoder angleEncoder;
    
    private double angleOffset;

    private double lastAngle;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS, Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        this.angleEncoder = new CANcoder(moduleConstants.cancoderID,CanBus.CanBusCvore);
        configAngleEncoder();

        /* Drive Motor Config */
        this.driveMotor = new TalonFX(moduleConstants.driveMotorID,CanBus.CanBusCvore);
        configDriveMotor();

        /* Angle Motor Config */
        this.angleMotor = new TalonFX(moduleConstants.angleMotorID,CanBus.CanBusCvore);
        configAngleMotor();

        this.lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            this.driveMotor.setControl(new DutyCycleOut(percentOutput, true, true));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
//            this.driveMotor.setControl(new VelocityDutyCycle, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            this.driveMotor.setControl(new VelocityDutyCycle(velocity, true, feedforward.calculate(desiredState.speedMetersPerSecond), 
                                        0, true));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle.getDegrees();   // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        this.angleMotor.setControl(new PositionDutyCycle(Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO))); 
        this.lastAngle = angle;
    }

    //  private void waitForCanCoder(){
    //     /*
    //      * Wait for CanCoder. (up to 1000ms)
    //      *
    //      * preventing race condition during program startup
    //      */
    //     for (int i = 0; i < 100; ++i) {
    //         angleEncoder.getAbsolutePosition();
    //         if (angleEncoder.getLastError() == ErrorCode.OK) {
    //             break;
    //         }
    //         Timer.delay(0.010);            
    //         CANcoderInitTime += 10;
    //     }
    // }


    public void resetToAbsolute() {
        // waitForCanCoder();
        double absolutePosition = Conversions.degreesToFalcon(this.getCanCoder().getDegrees() - angleOffset, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        this.angleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        this.angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        this.angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANCoderConfig);
    }

    private void configAngleMotor() {
        this.angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleTalonFXConfig);
        this.angleMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        // this.angleMotor.setconsetNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
        Timer.delay(.1);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        this.driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        this.driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        // this.driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        this.driveMotor.setRotorPosition(0, .1);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(360 * angleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(this.driveMotor.getRotorVelocity().getValue(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(this.angleMotor.getRotorPosition().getValue(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getRotorPosition().getValue(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(this.angleMotor.getRotorPosition().getValue(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
    };
}
