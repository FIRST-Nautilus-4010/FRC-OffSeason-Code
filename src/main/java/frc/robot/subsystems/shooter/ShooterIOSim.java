package frc.robot.subsystems.shooter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

    private final AbstractDriveTrainSimulation driveSimulation;

    private double velocityRps = 0.0;
    private double voltageApplied = 0.0;

    // Cuántos fuels recogidos están listos para lanzar
    private int fuelCount = 0;

    private int ballsPerShot = 2;
    private double shootRatePerSecond = 15;
    private double shootCooldownSeconds = 0.0;
    private double lastTimestamp = -1.0; 

    public ShooterIOSim(AbstractDriveTrainSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    // --------------------------------------------------------------------
    // API pública: el IntakeIOSim llama esto cuando recoge un fuel
    // --------------------------------------------------------------------

    /**
     * Notifica al shooter que el intake recogió un fuel.
     * Incrementa el contador de fuels disponibles para lanzar.
     */
    public void addFuel() {
        if (fuelCount < 42) {
            fuelCount++;
        }
    }

    public boolean isFull() {
        return fuelCount >= 42;
    }

    /** Retorna cuántos fuels hay disponibles para lanzar. */
    public int getFuelCount() {
        return fuelCount;
    }

    private void updateFuelVisualization() {
        var robotPose = driveSimulation.getSimulatedDriveTrainPose();
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );

        Pose3d[] fuelPoses = new Pose3d[fuelCount];

        // Dimensiones del cubo
        int cols  = 3;   // X → ancho del robot
        int rows  = 6;   // Y → profundidad del robot
        double spacing = 0.12; // separación entre pelotas (metros)

        // Offset para centrar el cubo dentro del robot
        double offsetX = -((cols - 1) * spacing) / 2.0 + 0.2;
        double offsetY = -((rows - 1) * spacing) / 2.0;
        double offsetZ = 0.3; // altura del piso del robot

        for (int i = 0; i < fuelCount; i++) {
            // Descomponer índice en 3D
            int x = i % cols;                  // columna
            int y = (i / cols) % rows;         // fila (profundidad)
            int z = i / (cols * rows);         // capa (altura)

            Pose3d localPose = new Pose3d(
                offsetX + x * spacing,
                offsetY + y * spacing,
                offsetZ + z * spacing,
                new Rotation3d()
            );

            fuelPoses[i] = robotPose3d.transformBy(
                new Transform3d(
                    localPose.getTranslation(),
                    localPose.getRotation()
                )
            );
        }

        Logger.recordOutput("Shooter/FuelInRobot", fuelPoses);
    }

    // --------------------------------------------------------------------
    // IO
    // --------------------------------------------------------------------

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Calcular deltaTime real
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = (lastTimestamp < 0) ? 0.02 : (now - lastTimestamp);
        lastTimestamp = now;

        // Reducir cooldown
        if (shootCooldownSeconds > 0) {
            shootCooldownSeconds -= dt;
        }

        // Disparar si: velocidad OK + hay fuel + cooldown listo
        boolean canShoot = Math.abs(velocityRps) > ShooterConfig.MIN_SHOOT_VELOCITY_RPS
                        && fuelCount >= ballsPerShot
                        && shootCooldownSeconds <= 0;

        if (canShoot) {
            for (int i = 0; i < ballsPerShot; i++) {
                shootFuel(i); // ← pasa el índice
                fuelCount--;
            }
            // Cooldown basado en pelotas/segundo: disparamos `ballsPerShot` pelotas
            shootCooldownSeconds = (double) ballsPerShot / shootRatePerSecond;
        }

        inputs.shooterData = new ShooterIOData(
            true,
            0.0,
            0.0,
            velocityRps * 2.0 * Math.PI,
            velocityRps * 2.0 * Math.PI,
            voltageApplied,
            voltageApplied,
            0.0,
            0.0
        );

        updateFuelVisualization();
    }

    @Override
    public void setVelocity(double velocityRps) {
        this.velocityRps = velocityRps;
        // Feedforward simple para telemetría
        double targetRadPerSec = velocityRps * 2.0 * Math.PI;
        this.voltageApplied = Math.min(12.0, Math.max(-12.0,
            ShooterConfig.VEL_KS * Math.signum(targetRadPerSec) +
            ShooterConfig.VEL_KV * targetRadPerSec
        ));
    }

    @Override
    public void setVoltage(double volts) {
        this.voltageApplied = Math.min(12.0, Math.max(-12.0, volts));
        // Estimación inversa de velocidad desde voltaje
        this.velocityRps = (volts - ShooterConfig.VEL_KS * Math.signum(volts))
                         / (ShooterConfig.VEL_KV * 2.0 * Math.PI);
    }

    @Override
    public void runOpenLoop(double output) {
        setVoltage(12.0 * output);
    }

    @Override
    public void stop() {
        this.velocityRps = 0.0;
        this.voltageApplied = 0.0;
    }

    // --------------------------------------------------------------------
    // LANZAMIENTO
    // --------------------------------------------------------------------

    private void shootFuel(int index) {
        try {
            var pose = driveSimulation.getSimulatedDriveTrainPose();

            // Offset lateral entre pelotas (en metros)
            double lateralSpacing = 0.15; // 15 cm entre pelotas
            // Centrar el grupo: si son 2 → offsets -0.075 y +0.075
            double lateralOffset = (index - (ballsPerShot - 1) / 2.0) * lateralSpacing;

            // Convertir offset lateral al frame del campo según heading del robot
            double heading = pose.getRotation().getRadians();
            Translation2d shooterOffset = new Translation2d(
                -Math.sin(heading) * lateralOffset,   // componente X
                Math.cos(heading) * lateralOffset    // componente Y
            );

            RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                pose.getTranslation().plus(shooterOffset), // ← posición desplazada
                new Translation2d(),
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                pose.getRotation(),
                Units.Meters.of(ShooterConfig.SHOOTER_HEIGHT_METERS),
                Units.MetersPerSecond.of(ShooterConfig.SHOOTER_EXIT_VELOCITY_MPS),
                Units.Degrees.of(ShooterConfig.SHOOTER_ANGLE_DEGREES)
            );

            // ... callbacks igual que antes ...

            SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        } catch (Exception e) {
            DriverStation.reportError("ShooterIOSim: error al lanzar fuel - " + e.getMessage(), false);
        }
    }
}