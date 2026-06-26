package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.shooter.ShooterIOSim;

public class IntakeIOSim implements IntakeIO{
    private final DCMotor pivotGearbox =
        DCMotor.getKrakenX60(2).withReduction(IntakeConfig.PIVOT_REDUCTION);

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        pivotGearbox,
        1.0,
        IntakeConfig.MOMENT_OF_INERTIA_KG_M2,
        IntakeConfig.COM_LENGTH_METERS,
        IntakeConfig.PIVOT_SOFT_LIMIT_REVERSE,
        IntakeConfig.PIVOT_SOFT_LIMIT_FORWARD,
        true,
        IntakeConfig.START_POS,
        IntakeConfig.STD_DEVS_PIVOT,
        0.001
    );


    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
        70,
        0,
        0.03,
        new Constraints(IntakeConfig.MAGIC_MOTION_VELOCITY_STR, IntakeConfig.MAGIC_MOTION_ACCELERATION_STR)
    );

    private double voltageApplied = 0;

    private final IntakeSimulation intakeSimulation;

    private final ShooterIOSim shooterIOSim;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain, ShooterIOSim shooterIOSim) {
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(0.570),
            Meters.of(0.305),
            IntakeSimulation.IntakeSide.FRONT,
            42
        );

        this.shooterIOSim = shooterIOSim;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        

        pivotSim.update(0.02);
        inputs.intakeData = new IntakeIOData(
             true, // motorConnected
             pivotSim.getAngleRads(),    // positionPivot
             pivotSim.getVelocityRadPerSec(),    // velocityPivot
             0,    // positionSpin
             0,    // velocitySpin
             voltageApplied,    // appliedVoltsPivotLeft
             voltageApplied,    // appliedVoltsPivotRight
             0,    // appliedVoltsSpinLeft
             0,    // appliedVoltsSpinRight
             0,    // torqueCurrentPivotLeft
             0,    // torqueCurrentPivotRight
             0,    // torqueCurrentSpinLeft
             0,    // torqueCurrentSpinRight
             pivotSim.getCurrentDrawAmps(),    // supplyCurrentPivotLeft
             pivotSim.getCurrentDrawAmps(),    // supplyCurrentPivotRight
             0,    // supplyCurrentSpinLeft
             0,    // supplyCurrentSpinRight
             20,   // temperaturePivotLeft (°C)
             20,   // temperaturePivotRight (°C)
             20,   // temperatureSpinLeft (°C)
             20    // temperatureSpinRight (°C)
         );
     }

     @Override
     public void setPivotVoltage(double volts) {
        pivotSim.setInputVoltage(volts);
        voltageApplied = volts;
     }

     @Override
     public void setSpinVoltage(double volts) {
        // Simulación: no hacemos nada con la salida.
     }

     @Override
     public void runOpenLoopPivot(double output) {
        setPivotVoltage(12 * output);
     }

     @Override
     public void runOpenLoopSpin(double output) {
         // Simulación: no hacemos nada con la salida.
     }

     @Override
     public void setPosition(double positionRad) {
        double currentAngleRad = pivotSim.getAngleRads();
        pivotPID.enableContinuousInput(-Math.PI, Math.PI);
        
        double voltage = pivotPID.calculate(currentAngleRad, positionRad);

        voltage = Math.min(12, Math.max(-12, voltage));

        setPivotVoltage(voltage);
     }

     @Override
    public void setVelocity(double velocityRps) {
        if (velocityRps != 0.0) {
            // Solo activar el intake si el shooter no está lleno
            if (!shooterIOSim.isFull()) {
                intakeSimulation.startIntake();
                int collected = intakeSimulation.getGamePiecesAmount();
                for (int i = 0; i < collected; i++) {
                    if (shooterIOSim.isFull()) break; // dejar de transferir si se llenó
                    shooterIOSim.addFuel();
                    intakeSimulation.obtainGamePieceFromIntake();
                }
            } else {
                intakeSimulation.stopIntake(); // detener si ya está lleno
            }
        } else {
            intakeSimulation.stopIntake();
        }
    }

     @Override
     public void stop() {
        setPivotVoltage(0);
     }
    
}
