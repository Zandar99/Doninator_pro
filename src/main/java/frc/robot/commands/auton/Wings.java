package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.commands.telescoping.SetMid;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class Wings extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public Wings (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2) {
        
        addCommands(
            new DropCone(drivetrain, intake, telescope, arm),
            new JustStopIntake(intake).withTimeout(0.1),
            new ParallelCommandGroup(
                new JustOuttake(intake).withTimeout(5.2),
                new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            ),
            new armLow(arm).withTimeout(.3),
            new JustStopIntake(intake).withTimeout(0.1),
            new armMid(arm).withTimeout(.2),
            new RunPathPlannerTrajectory2(drivetrain, trajectory2),
            new ParallelCommandGroup(
                new armMid(arm).withTimeout(0.7),
                new SetHigh(telescope).withTimeout(2)
            ),
            new JustIntake(intake).withTimeout(0.2),
            // new SetLow(telescope).withTimeout(2),
            // new armLow(arm).withTimeout(0.2),
            new JustStopIntake(intake).withTimeout(0.1)


        );
    }
    
}