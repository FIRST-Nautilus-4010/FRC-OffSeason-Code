package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final String GIT_SHA = "atun";
  private static final boolean replay = false; // Set to true to enable replay mode (runs from a log file)

  public Robot() {
    Logger.recordMetadata("GitSHA", GIT_SHA);

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else if (replay) {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {
        Logger.addDataReceiver(new NT4Publisher()); // Log to a folder on the computer
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    Logger.recordOutput("Mechanism3d/Measured/ChassisFixed", new Pose3d()); // Record the robot code version as an output

    m_robotContainer = new RobotContainer(Robot.isSimulation());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.disable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.subsystemManager.scheduleState(RobotState.INTAKE);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.initializeTeleOp();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
      SimulatedArena.getInstance().placeGamePiecesOnField();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    // Get the positions of the fuel (both on the field and in the air)
      Pose3d[] fuelPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Fuel");
      // Publish to telemetry using AdvantageKit
      Logger.recordOutput("FieldSimulation/Fuel", fuelPoses);
  }
}
