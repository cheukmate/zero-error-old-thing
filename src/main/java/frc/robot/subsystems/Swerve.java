package frc.robot.subsystems;
// also not used




import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;

    private Field2d field;

    public Swerve() {
     
        //gyro.calibrate();

        // gyro.configFactoryDefault();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod1.constants),
            new SwerveModule(1, Constants.Swerve.Mod2.constants),
            new SwerveModule(2, Constants.Swerve.Mod3.constants),
            new SwerveModule(3, Constants.Swerve.Mod4.constants)
        };

        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

                SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                
                if ((!gyro.isCalibrating()) || (!fieldRelative)) {
                    for (SwerveModule mod : mSwerveMods) {
                        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
                    }
                }
            }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }


    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), getPose());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void resetMotors() {
        for (SwerveModule mod : mSwerveMods) {
            mSwerveMods[mod.moduleNumber].resetToAbsolute();
        }
    }

    

    
    

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getModulePosition();
        }
        return positions;
    }

    private Rotation2d gyroYawOffset = Rotation2d.fromDegrees(0.0);
    public void zeroGyro() {
         //gyro.calibrate();
         //gyro.zeroYaw();
        gyroYawOffset = getYawIgnoringOffset();
    }

    private Rotation2d getYawIgnoringOffset() {
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    public Rotation2d getYaw() {
        if (gyro.isCalibrating()) {
            return Rotation2d.fromDegrees(0);
        } else {
            final double gyro_yaw = getYawIgnoringOffset().minus(gyroYawOffset).getDegrees();
            return (Constants.Swerve.invertGyro)
                    ? Rotation2d.fromDegrees(- gyro_yaw)
                    : Rotation2d.fromDegrees(gyro_yaw);
        }
    }
   
    

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        field.setRobotPose(getPose());

        SmartDashboard.putBoolean("GyroIsCalibrating", gyro.isCalibrating());
        SmartDashboard.putNumber("GyroYawWithOffset", getYaw().getDegrees());
        SmartDashboard.putNumber("GyroYawIgnoringOffset", getYawIgnoringOffset().getDegrees());
        for (SwerveModule mod : mSwerveMods) {
            mod.printToDash();
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Cancoder_Deg", mod.getCanCoder().getDegrees());
                    // System.out.println("Mod " + mod.moduleNumber + " Cancoder_Deg: " + mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Integrated_Deg", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Velocity_MetersPerSec", mod.getState().speedMetersPerSecond);
        }
    }
}