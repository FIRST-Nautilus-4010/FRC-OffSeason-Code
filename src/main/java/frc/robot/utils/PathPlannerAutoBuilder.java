package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.swerve.PoseTracker;

public class PathPlannerAutoBuilder {
    static RobotConfig config;
    static PoseTracker poseTracker;

    public static void initialize (SubsystemManager subsystemManager) {
        poseTracker = subsystemManager.getPoseTracker();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
            poseTracker::getPose, // Robot pose supplier
            poseTracker::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            poseTracker.getSwerve()::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> poseTracker.getSwerve().drive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(3, 0, 0.2), // Translation PID constants
                    new PIDConstants(2.2, 0, 0.2) // Rotation PID constants
            ),
            config,
            () -> {

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            poseTracker.getSwerve()
        );

        /* Aquí se registran los comandos nombrados */
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> subsystemManager.scheduleState(RobotState.INTAKE)));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> subsystemManager.scheduleState(RobotState.SHOOT)));
        NamedCommands.registerCommand("Travel", new InstantCommand(() -> subsystemManager.scheduleState(RobotState.TRAVEL)));
    }
    
}
