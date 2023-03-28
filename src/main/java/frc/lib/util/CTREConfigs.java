package frc.lib.util;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenixpro.signals.*;
import com.ctre.phoenixpro.spns.*;
import com.ctre.phoenixpro.hardware.*;
import com.ctre.phoenixpro.hardware.core.*;
import com.ctre.phoenixpro.configs.jni.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.wpiutils.*;

import frc.robot.Constants;

public final class CTREConfigs {
    
    public TalonFXConfiguration swerveDriveTalonFXConfig;
    public TalonFXConfiguration swerveAngleTalonFXConfig;
    public CANcoderConfiguration swerveCANCoderConfig;
    

    public CTREConfigs () {
        this.swerveDriveTalonFXConfig   = new TalonFXConfiguration();
        this.swerveAngleTalonFXConfig   = new TalonFXConfiguration();
        this.swerveCANCoderConfig       = new CANcoderConfiguration();
    
        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveDrivetrain.ANGLE_PEAK_CL;
        angleSupplyLimit.SupplyCurrentThreshold = Constants.SwerveDrivetrain.ANGLE_CONTINUOUS_CL;
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT;
        angleSupplyLimit.SupplyTimeThreshold = Constants.SwerveDrivetrain.ANGLE_PEAK_CURRENT_DURATION;

        this.swerveAngleTalonFXConfig.Slot0.kP = Constants.SwerveDrivetrain.ANGLE_kP;
        this.swerveAngleTalonFXConfig.Slot0.kI = Constants.SwerveDrivetrain.ANGLE_kI;
        this.swerveAngleTalonFXConfig.Slot0.kD = Constants.SwerveDrivetrain.ANGLE_kD;
        this.swerveAngleTalonFXConfig.Slot0.kV = Constants.SwerveDrivetrain.ANGLE_kF;
        // this.swerveAngleTalonFXConfig.CurrentLimits = angleSupplyLimit;

        // this.swerveAngleTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        //this.swerveCANCoderConfig.MagnetSensor = SensorInitializationStrategy.;


        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
            // Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            // Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            // Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            // Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        this.swerveDriveTalonFXConfig.Slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        this.swerveDriveTalonFXConfig.Slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        this.swerveDriveTalonFXConfig.Slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
        this.swerveDriveTalonFXConfig.Slot0.kV = Constants.SwerveDrivetrain.DRIVE_kF;
        
        //this.swerveDriveTalonFXConfig.SensorInitializationStrategy = SensorInitializationStrategy.BootToZero;
        this.swerveDriveTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        this.swerveDriveTalonFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;

        driveSupplyLimit.SupplyCurrentLimit = Constants.SwerveDrivetrain.DRIVE_PEAK_CL;
        driveSupplyLimit.SupplyCurrentThreshold = Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL;
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT;
        driveSupplyLimit.SupplyTimeThreshold = Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION;
        
        
        /* Swerve CANCoder Configuration */
        // this.swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // this.swerveCANCoderConfig.sensorDirection = Constants.SwerveDrivetrain.CAN_CODER_INVERTED;
        // this.swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // this.swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        this.swerveCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        this.swerveCANCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveDrivetrain.CAN_CODER_INVERTED;
        // this.swerveCANCoderConfig. = SensorInitializationStrategy.BootToAbsolutePosition;
        //this.swerveCANCoderConfig.MagnetSensor.wait(30, 0);
    
    }

}
