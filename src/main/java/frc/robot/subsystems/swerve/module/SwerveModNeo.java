package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import lib.factories.SparkMaxFactory;
import lib.utils.Rev.SparkMaxConfigs;
import lib.utils.drivers.CTREUtil;
import lib.utils.drivers.RevUtil;

public class SwerveModNeo {
  public final int moduleNumber;
  private static final int TIMEOUT_MILLISECONDS = 600;

  private final CANSparkMax m_driveMotor;
  private final RelativeEncoder m_driveEncoder;
  private final SparkMaxPIDController m_drivePID;

  private final CANSparkMax m_angleMotor;
  private final RelativeEncoder m_angleEncoder;
  private final SparkMaxPIDController m_anglePID;
  
  private final CANCoder m_canCoder;
  private final double m_canCoderOffsetDegrees;

  private double m_lastAngle;
  private boolean m_invert;
  private boolean m_openLoop = false;

  private SimpleMotorFeedforward m_driveFeedforward;

  public SwerveModNeo(int moduleNumber, double offsets, int[] canIds, boolean driveInvert) {
    this.moduleNumber = moduleNumber;
    this.m_invert = driveInvert;
    SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
    config.setCurrentLimit(50);

    m_driveMotor = new CANSparkMax(canIds[0], CANSparkMaxLowLevel.MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePID = m_driveMotor.getPIDController();

    m_angleMotor = new CANSparkMax(canIds[1], CANSparkMaxLowLevel.MotorType.kBrushless);
    m_angleEncoder = m_angleMotor.getEncoder();
    m_anglePID = m_angleMotor.getPIDController();

    m_canCoder = new CANCoder(canIds[2]);
    m_canCoderOffsetDegrees = offsets;

    configureDevices();
    m_lastAngle = getState().angle.getRadians();

    m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.DRIVE_KS, ModuleConstants.DRIVE_KV, ModuleConstants.DRIVE_KA);
  }

  public void setDesiredState(SwerveModuleState state) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state, getState().angle);

    double speed = state.speedMetersPerSecond;
    SmartDashboard.putNumber("Desired Drive Speed " + moduleNumber, speed);

    if (m_openLoop) {
      m_driveMotor.set(speed / ModuleConstants.MAX_SPEED_MPS);
    } else {
      double speedff = m_driveFeedforward.calculate(speed);
      m_drivePID.setReference(speed, CANSparkMax.ControlType.kVoltage, 0, speedff);
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= 0.0
      ? m_lastAngle
      : state.angle.getRadians();

    SmartDashboard.putNumber("Actual Drive Speed " + moduleNumber, m_driveEncoder.getVelocity());
    SmartDashboard.putNumber("Drive Speed Percent " + moduleNumber, m_driveMotor.getAppliedOutput());

    m_anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
    m_lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = m_driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(m_angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public double getAngle() {
    return Units.radiansToDegrees(m_angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(m_angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
  }

  public double getTargetAngle(){
    return m_lastAngle;
  }

  public void resetToAbsolute(){
    autoRetry(() -> m_angleEncoder.setPosition(Units.degreesToRadians(getCanCoder().getDegrees() - m_canCoderOffsetDegrees)));
  }

  /**
   * Gives best effort to a CTRE API call
   * @param callback A function that is used to make the configuration goes smoothly
   */
  private void autoRetry(CTREUtil.ConfigCall callback) {
    CTREUtil.autoRetry(callback);
  }

  /**
   * Gives best effort to a Rev API call
   * @param callback A function that is used to make the configuration goes smoothly
   */
  private void autoRetry(RevUtil.ConfigCall callback) {
    RevUtil.autoRetry(callback);
  }

  private void configureDevices() {
    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    canCoderConfiguration.sensorDirection = false;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    

    autoRetry(() -> m_canCoder.configFactoryDefault()) ;
    autoRetry(() -> m_canCoder.configAllSettings(canCoderConfiguration, TIMEOUT_MILLISECONDS));
    autoRetry(() -> m_canCoder.configMagnetOffset(0, TIMEOUT_MILLISECONDS));

    // Drive motor configuration.
    autoRetry(() -> m_driveMotor.restoreFactoryDefaults());
    m_driveMotor.setInverted(m_invert);
    autoRetry(() -> m_driveMotor.setIdleMode(IdleMode.kBrake));
    autoRetry(() -> m_driveMotor.setOpenLoopRampRate(0.5));


    autoRetry(() -> m_driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_CONVERSION_FACTOR));
    autoRetry(() -> m_driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_CONVERSION_FACTOR / 60));
    autoRetry(() -> m_driveEncoder.setPosition(0));
    SparkMaxConfigs.configCanStatusFrames(m_driveMotor);

    autoRetry(() -> m_drivePID.setP(0.5));

    // Angle motor configuration.
    autoRetry(() -> m_angleMotor.restoreFactoryDefaults());
    m_angleMotor.setInverted(true);
    autoRetry(() -> m_angleMotor.setIdleMode(IdleMode.kBrake));
    autoRetry(() -> m_angleMotor.setSmartCurrentLimit(40));

    autoRetry(() -> m_anglePID.setP(ModuleConstants.MODULE_KP));
    autoRetry(() -> m_anglePID.setI(0.0));
    autoRetry(() -> m_anglePID.setD(ModuleConstants.MODULE_KD));
    autoRetry(() -> m_anglePID.setFF(0.0));

    autoRetry(() -> m_anglePID.setPositionPIDWrappingEnabled(true));
    autoRetry(() -> m_anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI));
    autoRetry(() -> m_anglePID.setPositionPIDWrappingMinInput(0));

    autoRetry(() -> m_angleEncoder.setPositionConversionFactor(ModuleConstants.POSITION_CONVERSION_FACTOR));
    autoRetry(() -> m_angleEncoder.setPosition(Units.degreesToRadians(m_canCoder.getAbsolutePosition() - m_canCoderOffsetDegrees)));
    SparkMaxConfigs.configCanStatusFrames(m_angleMotor);
  }
}
