package frc.robot.subsystems.swerve.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;

public class Drive extends Command {

    private static final double LIN_DEADZONE =
            0.1 * SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND;

    private static final double ANG_DEADZONE =
            0.1 * SwerveConfig.MAX_ANG_SPD;

    private final Swerve swerve;
    private final PoseTracker poseTracker;

    /* ---------- Joystick ---------- */

    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> omegaInput;
    private final Supplier<Boolean> resetYaw;

    /* ---------- Assist flags ---------- */

    public static boolean assistX;
    public static boolean assistY;
    public static boolean assistTheta;
    public static boolean aimEnabled;
    public static boolean velocityHeadingEnabled;
    public static boolean isSim = false;

    public static boolean onAimTolerance = false;

    /* ---------- Targets ---------- */

    // Target de traslación (cuando assistX o assistY están activos)
    public static Pose2d targetPose;

    // Punto al que se quiere apuntar (solo usado si aimEnabled = true)
    private final Supplier<Pose2d> aimPose;

    private final HolonomicDriveController controller;


    public Drive(
            Swerve swerve,
            PoseTracker poseTracker,

            Supplier<Double> xInput,
            Supplier<Double> yInput,
            Supplier<Double> omegaInput,
            Supplier<Boolean> resetYaw,

            Supplier<Pose2d> aimPose

    ) {

        this.swerve = swerve;
        this.poseTracker = poseTracker;

        this.xInput = xInput;
        this.yInput = yInput;
        this.omegaInput = omegaInput;
        this.resetYaw = resetYaw;
        this.aimPose = aimPose;

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        SwerveConfig.P_Z,
                        SwerveConfig.I_Z,
                        SwerveConfig.D_Z,
                        SwerveConfig.Z_CONTROLER);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if (isSim) {
            controller = new HolonomicDriveController(
                new PIDController(
                        10,
                        SwerveConfig.I_X,
                        0.1),
                new PIDController(
                        10,
                        SwerveConfig.I_Y,
                        0.1),
                thetaController
            );
        } else {
            controller = new HolonomicDriveController(
                new PIDController(
                        SwerveConfig.P_X,
                        SwerveConfig.I_X,
                        SwerveConfig.D_X),
                new PIDController(
                        SwerveConfig.P_Y,
                        SwerveConfig.I_Y,
                        SwerveConfig.D_Y),
                thetaController
            );
        }
        

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        Pose2d currentPose = poseTracker.getPose();
        Pose2d baseTarget = targetPose;

        // Si no hay objetivo de traslación, usa la pose actual
        if (baseTarget == null) {
            baseTarget = currentPose;
        }

        /* ---------------- Entradas manuales ---------------- */

        double manVx = xInput.get() * SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND;
        double manVy = yInput.get() * SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND;
        double manOmega = omegaInput.get() * SwerveConfig.MAX_ANG_SPD;

        manVx = applyDeadzone(manVx, LIN_DEADZONE);
        manVy = applyDeadzone(manVy, LIN_DEADZONE);
        manOmega = applyDeadzone(manOmega, ANG_DEADZONE);

        /* ---------------- Rotación deseada para el controlador ---------------- */

        Rotation2d desiredRotation = new Rotation2d();

        double vx = manVx;
        double vy = manVy;

        double scale = 1.0;

        if (aimEnabled) {

            /* ---------------- ERROR ANGULAR ---------------- */

            double angleError =
                desiredRotation.minus(currentPose.getRotation()).getRadians();

            // Normalizar a [-pi, pi]
            angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

            // Escala suave (coseno)
            double angleScale = Math.cos(angleError);

            /* ---------------- CAPACIDAD ANGULAR ---------------- */

            double maxOmega = SwerveConfig.MAX_ANG_SPD;

            double timeToCorrect = Math.abs(angleError) / maxOmega;

            double speed = Math.hypot(vx, vy);

            // evita división por cero
            double timeToTravel = 1.0 / Math.max(speed, 0.01);

            double capabilityScale = 1.0 - Math.min(1.0, timeToCorrect / timeToTravel);

            /* ---------------- COMBINACIÓN ---------------- */

            scale = Math.min(angleScale, capabilityScale);

            // Clamp para que no muera el robot
            scale = Math.max(0.25, scale);

            desiredRotation = computeAimRotation(currentPose);
        } else if (assistTheta) {
            desiredRotation = baseTarget.getRotation();
        } else if (velocityHeadingEnabled) {
            desiredRotation = computeVelocityHeading(currentPose);
        }

        Pose2d poseForController =
                new Pose2d(baseTarget.getTranslation(), desiredRotation);

        /* ---------------- Salida automática ---------------- */

        ChassisSpeeds auto = controller.calculate(
                currentPose,
                poseForController,
                SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND,
                desiredRotation
        );

        /* ---------------- Mezcla por asistencia ---------------- */

        if (assistX) {
            vx = auto.vxMetersPerSecond;
        }

        if (assistY) {
            vy = auto.vyMetersPerSecond;
        }

        double omega;

        if (assistTheta) {
            omega = auto.omegaRadiansPerSecond;

        } else {
            omega = manOmega;
        }

        /* ---------------- Drive (siempre field-relative) ---------------- */

        swerve.driveFieldRelative(vx * scale, vy * scale, omega);

        // Opción para resetear yaw del gyro (por ejemplo, botón en el joystick).
        if (resetYaw.get()) {
            swerve.zeroHeading();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    /* ------------------------------------------------------------------ */

    private Rotation2d computeAimRotation(Pose2d currentPose) {

        if (aimPose.get() == null || aimPose.get() == null) {
            return currentPose.getRotation();
        }

        Pose2d aim = aimPose.get();

        double dx = aim.getX() - currentPose.getX();
        double dy = aim.getY() - currentPose.getY();

        if (Math.abs(Math.atan2(dy, dx) - currentPose.getRotation().getRadians()) < 0.04) {
            onAimTolerance = true;
        } else {
            onAimTolerance = false;
        }

        return new Rotation2d(Math.atan2(dy, dx));
    }

    private Rotation2d computeVelocityHeading(Pose2d currentPose) {
        return new Rotation2d(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));
    }

    private static double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }
}