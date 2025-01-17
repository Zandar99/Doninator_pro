// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.macros.mid;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.AllignedSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.utils.TrajectoryHelper;
import frc.robot.commands.auton.Basic;
import frc.robot.commands.auton.Center;
import frc.robot.commands.auton.CenterBeta;
import frc.robot.commands.auton.RA1;
import frc.robot.commands.auton.RA100;
import frc.robot.commands.auton.RA200;
import frc.robot.commands.auton.Wings;

public class RobotContainer {


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  public static final PS4Controller operator = new PS4Controller(1);

  public static boolean manualControls;

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  // private final JoystickButton driveSlow = new JoystickButton(driver, PS4Controller.Button.kR2.value);


  // private final JoystickButton shoot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonUp = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonMid = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton pneumaticButtonDown = new JoystickButton(operator, PS4Controller.Button.kSquare.value);

  private final JoystickButton outtakeButton = new JoystickButton(operator, PS4Controller.Button.kL2.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, PS4Controller.Button.kR2.value);
  private final JoystickButton intakeIn = new JoystickButton(driver, PS4Controller.Button.kShare.value);
  private final JoystickButton intakeOut = new JoystickButton(driver, PS4Controller.Button.kOptions.value);


  private final JoystickButton telescopeForward = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton telescopeBack = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton telescopePrint = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton telescopeMid = new JoystickButton(operator, PS4Controller.Button.kShare.value);
  private final JoystickButton telescopeHigh = new JoystickButton(operator, PS4Controller.Button.kOptions.value);


  private final JoystickButton candlePurple = new JoystickButton(operator, PS4Controller.Button.kL3.value);
  private final JoystickButton candleYellow = new JoystickButton(operator, PS4Controller.Button.kR3.value);
  private final JoystickButton candleIncrement = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton candleRainbow = new JoystickButton(driver, PS4Controller.Button.kCross.value);

  private final JoystickButton telescopeRun = new JoystickButton(operator, PS4Controller.Button.kPS.value);
  private final JoystickButton telescopeZero = new JoystickButton(operator, PS4Controller.Button.kCross.value);
  private final JoystickButton telescopePrint2 = new JoystickButton(driver, PS4Controller.Button.kShare.value);


  private final JoystickButton macroLow = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
  private final JoystickButton macroMid = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton macroHigh = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton macroHP = new JoystickButton(operator, PS4Controller.Button.kPS.value);
  private final JoystickButton macroTransport = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);

  private final JoystickButton operatorOptions = new JoystickButton(operator, PS4Controller.Button.kOptions.value);


  private final JoystickButton switchMode = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);
  // private final JoystickButton slowDown = new JoystickButton(driver, PS4Controller.Button.kR2.value);



  // private final JoystickButton telescopeZeroWHILE = new JoystickButton(driver, PS4Controller.Button.kPS.value);

  int operatorPOV = operator.getPOV();


  public double yAxis = -operator.getRawAxis(1);

  /* Deadbands */
  // yAxis = (Math.abs(yAxis) < DEADBAND) ? 0 : yAxis;




  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  private final ArmSubsystem arm = new ArmSubsystem(hub);
  private final IntakeSubsystem intake = new IntakeSubsystem(hub);
  private final frc.robot.subsystems.TelescopingSubsystem telescope = new frc.robot.subsystems.TelescopingSubsystem();
  private final OurBeautifulGlowingCANdleSubsystem candle = new OurBeautifulGlowingCANdleSubsystem();
  private final AllignedSubsystem macro = new AllignedSubsystem(arm, telescope, intake);

  public static PneumaticHub hub      = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Commands */
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  private final Command c_driveSlow = new InstantCommand( () -> drivetrain.driveSlow() );
  private final Command c_driveNormal = new InstantCommand( () -> drivetrain.driveNormal() );

  private final Command c_runPneumaticUp = new InstantCommand( () -> arm.setHigh());
  private final Command c_runPneumaticMid = new InstantCommand( () -> arm.setMid());
  private final Command c_runPneumaticDown = new InstantCommand( () -> arm.setLow());
  double intakePower = 0.5;
  public final Command c_runIntake = new InstantCommand( () -> intake.intakeDefSpeed());
  private final Command c_outtakeIntake = new InstantCommand( () -> intake.outtakeDefSpeed());
  private final Command c_stopIntake = new InstantCommand( () -> intake.stop());
  private final Command c_intakeOut = new InstantCommand( () -> intake.pneumaticExtract());
  private final Command c_intakeIn = new InstantCommand( () -> intake.pneumaticRetract());


  private final Command c_TelescopeForward = new InstantCommand( () -> telescope.forward());
  private final Command c_TelescopeBack = new InstantCommand( () -> telescope.backward());
  private final Command c_TelescopeStop = new InstantCommand( () -> telescope.stop());
  private final Command c_TelescopeReset = new InstantCommand( () -> telescope.resetMotors());
  private final Command c_TelescopePrintStuff = new InstantCommand( () -> telescope.printStuff());

  private final Command c_TelescopeLow = new InstantCommand( () -> telescope.runZero());
  private final Command c_TelescopeMid = new InstantCommand( () -> telescope.runMid());
  private final Command c_TelescopeHigh = new InstantCommand( () -> telescope.runHigh());
  private final Command c_TelescopeHP = new InstantCommand( () -> telescope.runHP());

  private final Command c_candleYellow = new InstantCommand( () -> candle.yellow());
  private final Command c_candlePurple = new InstantCommand( () -> candle.purple());
  private final Command c_candleIncrement = new InstantCommand( () -> candle.incrementColor(-5));
  private final Command c_candleRainbow = new InstantCommand( () -> candle.gsa());

  private final Command c_macroLow = new InstantCommand( () -> macro.ground());
  private final Command c_macroMid = new InstantCommand( () -> macro.mid());
  private final Command c_macroHigh = new InstantCommand( () -> macro.high());
  private final Command c_macroTransport = new InstantCommand( () -> macro.transport());
  private final Command c_macroHP = new InstantCommand( () -> macro.HP());
  private final Command c_macroStop = new InstantCommand( () -> macro.stop());

  private final Command c_switchMode = new InstantCommand( () -> switchMode());
  private final Command c_slowDown = new InstantCommand( () -> slowDown());




  // private final Command c_timedTelescope = new InstantCommand( () -> telescope.testRun());
  // private final Command c_telescopeZeroWHILE = new InstantCommand (() -> telescope.runZeroWhile());


  /* Trajectories */
  public PathPlannerTrajectory B_LW1, B_LW2, B_RW1, B_RW2, RL_STR1, RL_STR2, R_Center, B_Center, RR_STR1,
                               RR_STR2, R_CenterBeta1, R_CenterBeta2, BlueTurn, BRW1, BRW2, BLW1, BLW2, BLW1R, BLW2R;
  private enum Colors { None, Red, Blue }
  private enum Autons { Nothing, LeftWing, Center, RightWing, Test }
  private SendableChooser<Colors> colorChooser = new SendableChooser<>();
  private SendableChooser<Autons> autonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);

    yAxis = -operator.getRawAxis(1);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
    configureAuton();
    dashboardStuff();
    checkOperatorPOV();

    // UsbCamera cam = CameraServer.startAutomaticCapture();
    // cam.setResolution(320, 320);
    // cam.setFPS(30);

  }

  private void switchMode(){
    System.out.println("switch mode");
    
    manualControls = !manualControls;
  }

  public void configureButtonBindings(){
    zeroGyro.onTrue(c_zeroGyro);

    outtakeButton.onTrue(c_outtakeIntake);
    intakeButton.onTrue(c_runIntake);
    outtakeButton.onFalse(c_stopIntake);
    intakeButton.onFalse(c_stopIntake);
    telescopePrint2.onTrue(c_TelescopePrintStuff);

    candleYellow.onTrue(c_candleYellow);
    candlePurple.onTrue(c_candlePurple);
    candleRainbow.onTrue(c_candleRainbow);

    switchMode.onTrue(c_switchMode);

    intakeIn.onTrue(c_intakeIn);
    intakeOut.onTrue(c_intakeOut);

    // slowDown.onTrue(c_slowDown);
    // driveSlow.onFalse(c_driveNormal);

  }
  private void configureManualButtonBindings() {
    // new JoystickButton(driver, PS4Controller.Button.kTriangle.value).whileHeld(c_shoot);
    pneumaticButtonUp.onTrue(c_runPneumaticUp);
    pneumaticButtonMid.onTrue(c_runPneumaticMid);
    pneumaticButtonDown.onTrue(c_runPneumaticDown);

    telescopeForward.onTrue(c_TelescopeForward);
    telescopeBack.onTrue(c_TelescopeBack);

    telescopeForward.onFalse(c_TelescopeStop);
    telescopeBack.onFalse(c_TelescopeStop);
    telescopePrint.onTrue(c_TelescopePrintStuff);

    telescopeRun.onTrue(c_TelescopeLow);
    telescopeMid.onTrue(c_TelescopeMid);
    telescopeHigh.onTrue(c_TelescopeHigh);

  }

  private void configureAutoButtonBindings() {
    macroLow.onTrue(c_macroLow);
    macroMid.onTrue(c_macroMid);
    macroHigh.onTrue(c_macroHigh);
    macroHP.onTrue(c_macroHP);
    macroTransport.onTrue(c_macroTransport);
    macroTransport.onFalse(c_macroStop);
    macroLow.onFalse(c_macroStop);
    macroMid.onFalse(c_macroStop);
    macroHigh.onFalse(c_macroStop);
    macroHP.onFalse(c_macroStop);

  }


  public void checkOperatorPOV(){
    if(operator.getPOV() == 270){
      System.out.println("hello270");
      macro.ground();
    }
    else if(operator.getPOV() == 90){
      System.out.println("hello90");
      macro.mid();
    }
    else if(operator.getPOV() == 0){
      macro.high();
      System.out.println("hello0");
    }
    else if(operator.getPOV() == 180){
      macro.transport();
    }

    operatorOptions.onTrue(c_macroHP);
    operatorOptions.onFalse(c_macroStop);



  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveTeleop(
        drivetrain,
        driver,
        true, true
      )
    );

    // intake.setDefaultCommand(new IntakeCommand(intake, operator));

  }

  private void loadTrajectories() {
    // Test = TrajectoryHelper.loadWPILibTrajectoryFromFile("test1");
    double accel = 2;
    B_LW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BB_LW1",3.0,accel, false);
    B_LW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BB_LW2",3,accel, false);
    B_RW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BlueBasic",3,accel, false);
    B_RW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BB_RW2",3,accel, false);
    B_Center = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_Center",2.0,1, false);

    RL_STR1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RL_STR1",3.0,2, false);
    RL_STR2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RL_STR2",3.0,2, false);
    RR_STR1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RR_STR1",3.7,2, false);
    RR_STR2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RR_STR2",4,2, false);
    R_Center = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("R_Center",1.8,1, false);

    R_CenterBeta1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("R_CenterBeta1",3,2, false);
    R_CenterBeta2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("R_CenterBeta2",1.4,1, false);

    BlueTurn = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BlueTurn",2.5,2.2, false);

    BRW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BRW1",4,2.2, false);
    BRW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BRW2",4,2, false);
    BLW1R = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BRW1",4,2.2, true);
    BLW2R = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BRW2",4,2, true);

    BLW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BLW1",4,2.2, false);
    BLW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("BLW2",3,1.5, false);

  }

  private void configureAuton() {
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption("LeftWing", Autons.LeftWing);
    autonChooser.addOption("Center", Autons.Center);
    autonChooser.addOption("RightWing", Autons.RightWing);
    autonChooser.addOption("Test", Autons.Test);


    colorChooser.setDefaultOption("Nothing", Colors.None);
    colorChooser.addOption("RED", Colors.Red);
    colorChooser.addOption("BLUE", Colors.Blue);



    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    tab.add("Color Chooser", colorChooser);
    tab.add("Auton Chooser", autonChooser);
  }

  private void dashboardStuff () {
    ShuffleboardTab tab = Shuffleboard.getTab("LiftingArm");
    tab.addNumber("PSI", () -> hub.getPressure(0));
    tab.addNumber("Pitch: ", () -> drivetrain.getGyroPitchDegrees() );
    tab.addNumber("Yaw: ", () -> drivetrain.getGyroAngleDegrees());
    tab.addNumber("Roll: ", () -> drivetrain.getGyroRoll());

  }

  private void slowDown(){
    Constants.SwerveDrivetrain.MAX_SPEED = 1;
}

  public Command getAutonomousCommand() {

    switch (colorChooser.getSelected()){

      case None:
        return new WaitCommand(15.0);


      case Red:
        switch (autonChooser.getSelected()) {
          case Nothing:
            return new WaitCommand(15.0);

          case LeftWing:
            return new Wings(drivetrain, intake, telescope, arm, candle, RL_STR1, RL_STR2);
          
          case RightWing:
            return new Wings(drivetrain, intake, telescope, arm, candle, RR_STR1, RR_STR2);
        
          case Center:
            return new Center(drivetrain, intake, telescope, arm, candle, R_Center, null);



          case Test:
            // return new RA1(drivetrain, intake, telescope, arm, R_STR, B_STR, null);
            // return new RunPathPlannerTrajectory2(drivetrain, B_STR);
            // return new RA200(drivetrain, intake, telescope, arm, RR_STR1, RR_STR2);
            return new CenterBeta(drivetrain, intake, telescope, arm, R_CenterBeta1, R_CenterBeta2);
            
        
          default:
            return new WaitCommand(15.0);
        }


      case Blue:
        switch (autonChooser.getSelected()) {
          case Nothing:
            return new WaitCommand(15.0);

          case LeftWing:
            return new Wings(drivetrain, intake, telescope, arm, candle, BLW1, BLW2);
          
          case RightWing:
            // return new Wings(drivetrain, intake, telescope, arm, B_RW1, B_RW2);
            return new Wings(drivetrain, intake, telescope, arm, candle, BRW1, BRW2);
            //basic
        
          case Center:
            return new Center(drivetrain, intake, telescope, arm, candle, R_Center, null);

          case Test:
            // return new Basic(drivetrain, intake, telescope, arm, B_RW1, BlueTurn);
            // return new RunPathPlannerTrajectory2(drivetrain, BlueTurn);
            return new Wings(drivetrain, intake, telescope, arm, candle, BLW1R, BLW2R);

          default:
            return new WaitCommand(15.0);
        }

        default:
          return new WaitCommand(15.0);
    }
    
  }

  public void periodic(){
    switchMode.onTrue(c_switchMode);
    if(manualControls == true){
      configureManualButtonBindings();
    }
    else{
      configureAutoButtonBindings();
    }

  }

}
