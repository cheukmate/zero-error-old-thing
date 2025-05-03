package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final int driverPort = 0;
    public static final int operatorPort = 1;

    public static final class Swerve {
        public static final double stickDeadband = 0.4;

        public static final int pigeonID = 6;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23);
        public static final double wheelBase = Units.inchesToMeters(22);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (150 / 7.0); // 12.8:1

        // +x = forward
        // +y = left
        // public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        //     new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right (Module 1)
        //     new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // Back Right (Module 3)
        //     new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left (Module 4) 
        //     new Translation2d(wheelBase / 2.0, trackWidth / 2.0) // Front Left (Module 2)
        // );

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right (Module 1)
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // Front Left (Module 2)
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // Back Right (Module 3)
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0) // Back Left (Module 4) 
        );

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;
    

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5*1.6; // meters per second
        public static final double maxAngularVelocity = 11.5*1.6;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false; //cross check this
        public static final boolean angleInvert = false; //cross check this

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false; //cross check this

        /* Module Specific Constants */
        /* Module 1 */
        //frontright
        //drive motor = cansparkmax plugged in
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0); // CC1
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        
        /* Module 3 */
        //backright
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0); // CC2
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        

        /* Module 4 */
        //backleft
        public static final class Mod4 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 24;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0); // CC3
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Module 2 */
        //frontleft
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0); // CC4
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset); 
        }
        
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    }

    
    public static final double translationSlewRate = 3*10.0;
    public static final double strafeSlewRate = 3*10.0;
    public static final double rotationSlewRate = 3*1.0;
    public static final double kMaxPowerMultiplier = (1.2/0.45);
    



}