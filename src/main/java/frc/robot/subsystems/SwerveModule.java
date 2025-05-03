package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;

import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  private SparkMaxConfig driveConfig = new SparkMaxConfig();
  private SparkMaxConfig angleConfig = new SparkMaxConfig();

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private double getCanCoderDegreesMinusOffset() {
    return getCanCoder().getDegrees() - angleOffset.getDegrees();
  }

  public void resetToAbsolute() {
    // double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(getCanCoderDegreesMinusOffset());
  }

  private void configAngleEncoder() {
    //look for a way to restore factory defaults
    angleEncoder.getConfigurator().apply(CTREConfigs.swerveCanCoderConfig);
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal); -- look at this later!
  }

  private void configAngleMotor() {
    // new config :P
 
    angleConfig
      .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
      .inverted(Constants.Swerve.angleInvert) // should be true for the current chassis, check this if you reassemble.
      .idleMode(Constants.Swerve.angleNeutralMode)
      .voltageCompensation(Constants.Swerve.voltageComp);
    
    
    angleConfig.encoder
    .positionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleConfig.closedLoop
    .p(Constants.Swerve.angleKP)
    .i(Constants.Swerve.angleKI)
    .d(Constants.Swerve.angleKD); // RIP feedforward

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // hope these are right, never understood the entire persist and safe parameters things fully
    
    

//DEAD CODE
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    // angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    // angleMotor.setInverted(Constants.Swerve.angleInvert);
    // angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    // integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    // angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
    // angleController.setP(Constants.Swerve.angleKP);
    // angleController.setI(Constants.Swerve.angleKI);
    // angleController.setD(Constants.Swerve.angleKD);
    // angleController.setFF(Constants.Swerve.angleKFF);
    // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    // angleMotor.burnFlash();
    // resetToAbsolute();

  }

  private void configDriveMotor() {

    driveConfig
      .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
      .inverted(Constants.Swerve.driveInvert) // should be true for the current chassis, check this if you reassemble.
      .idleMode(Constants.Swerve.driveNeutralMode)
      .voltageCompensation(Constants.Swerve.voltageComp);
    
    
    driveConfig.encoder
    .positionConversionFactor(Constants.Swerve.driveConversionPositionFactor)
    .velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveConfig.closedLoop
    .p(Constants.Swerve.driveKP) 
    .i(Constants.Swerve.driveKI)
    .d(Constants.Swerve.driveKD); // RIP feedforward

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder.setPosition(0.0);


    // old and dead

    //driveMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    //driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
   // driveMotor.setInverted(Constants.Swerve.driveInvert);
    //driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    // driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    // driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    // driveMotor.burnFlash();
    
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
      feedforward.calculate(desiredState.speedMetersPerSecond);

      // driveController.setReference(
      //     desiredState.speedMetersPerSecond,
      //     ControlType.kVelocity,
      //     0,
      //     feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  public void printToDash() {
    SmartDashboard.putNumber("Mod " + moduleNumber + " Angle_Deg", getAngle().getDegrees());
    SmartDashboard.putNumber("Mod " + moduleNumber + " CanCoderMinusOffset_Deg", getCanCoderDegreesMinusOffset());
    SmartDashboard.putNumber("Mod " + moduleNumber + " DriveMotVel_MetersPerSec", driveEncoder.getVelocity());
    SmartDashboard.putNumber("Mod " + moduleNumber + " AngularMotVel_DegsPerSec", angleEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Mod " + moduleNumber + " DriveMotAppliedOutput", driveMotor.getAppliedOutput());
    SmartDashboard.putNumber("Mod " + moduleNumber + " AngularMotAppliedOutput", angleMotor.getAppliedOutput());
  }

  private void setAngle(SwerveModuleState desiredState) {
    printToDash();
    SmartDashboard.putNumber("Mod " + moduleNumber + " DesiredAngle_Deg", desiredState.angle.getDegrees());
    SmartDashboard.putNumber("Mod " + moduleNumber + " DesiredSpeed_MeterPerSec", desiredState.speedMetersPerSecond);
    
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    SmartDashboard.putNumber("Mod " + moduleNumber + " DesiredAngleRef_Deg", angle.getDegrees());
    
    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
}