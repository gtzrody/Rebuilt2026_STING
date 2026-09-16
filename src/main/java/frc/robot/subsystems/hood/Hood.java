package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Hood extends SubsystemBase {
  private static final int MOTOR_ID = 17;
  private static final double GEAR_RATIO = 36.0;
  private static final double KP = 30.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;

  private static final double MIN_ANGLE_DEG = 3.0;
  private static final double MAX_ANGLE_DEG = 130.0;

  private final TalonFX hoodFx = new TalonFX(MOTOR_ID, new CANBus("canivore"));
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  private static final double ARM_LENGTH_M = Feet.of(3).in(edu.wpi.first.units.Units.Meters);
  private static final double ARM_MASS_KG = Pounds.of(1).in(edu.wpi.first.units.Units.Kilograms);
  private static final double ARM_MOI = (ARM_MASS_KG * ARM_LENGTH_M * ARM_LENGTH_M) / 3.0;

  private final DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), ARM_MOI, GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2).per(Second),
              Volts.of(7),
              Seconds.of(4)),
          new SysIdRoutine.Mechanism(
              volts -> hoodFx.setVoltage(volts.in(Volts)),
              log -> {},
              this));

  public Hood() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 90.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kP = KP;
    config.Slot0.kI = KI;
    config.Slot0.kD = KD;

    config.MotionMagic.MotionMagicCruiseVelocity =
        DegreesPerSecond.of(90).in(edu.wpi.first.units.Units.RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        DegreesPerSecondPerSecond.of(45)
            .in(edu.wpi.first.units.Units.RotationsPerSecondPerSecond);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Degrees.of(MAX_ANGLE_DEG).in(edu.wpi.first.units.Units.Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Degrees.of(MIN_ANGLE_DEG).in(edu.wpi.first.units.Units.Rotations);

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

    hoodFx.getConfigurator().apply(config);

    hoodFx.setPosition(Degrees.of(0));
  }

  /**
   * Continuously commands the hood toward the requested angle.
   */
  public Command setAngle(Angle angle) {
    return run(() -> setAngleSetpoint(angle));
  }

  /**
   * Commands the hood to the requested angle and finishes after reaching tolerance.
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) {
    return Commands.sequence(
        runOnce(() -> setAngleSetpoint(angle)),
        Commands.waitUntil(() -> getAngle().isNear(angle, tolerance)));
  }

  /**
   * Sets the persistent hood angle setpoint.
   */
  public void setAngleSetpoint(Angle angle) {
    Angle clamped =
        Degrees.of(
            Math.max(
                MIN_ANGLE_DEG,
                Math.min(MAX_ANGLE_DEG, angle.in(Degrees))));

    positionRequest.withPosition(clamped);
    hoodFx.setControl(positionRequest);
  }

  /**
   * Manual duty-cycle control.
   */
  public Command set(double cycle) {
    return run(() -> {
      double output = Math.max(-1.0, Math.min(1.0, cycle));
      dutyCycleRequest.withOutput(output);
      hoodFx.setControl(dutyCycleRequest);
    });
  }

  public Angle getAngle() {
    // TalonFX is configured to expose mechanism-side position.
    return hoodFx.getPosition().getValue();
  }

  /**
   * Same SysId voltage/time profile as the original YAMS Arm.
   */
  public Command sysId() {
    Command dynamicForward =
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> getAngle().in(Degrees) >= MAX_ANGLE_DEG - 1.0)
            .withTimeout(3.0);

    Command dynamicReverse =
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> getAngle().in(Degrees) <= MIN_ANGLE_DEG + 1.0);

    Command quasistaticForward =
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> getAngle().in(Degrees) >= MAX_ANGLE_DEG - 1.0)
            .withTimeout(3.0);

    Command quasistaticReverse =
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> getAngle().in(Degrees) <= MIN_ANGLE_DEG + 1.0);

    return Commands.sequence(
        Commands.print("Starting SysId!"),
        dynamicForward,
        dynamicReverse,
        quasistaticForward,
        quasistaticReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle Degrees", getAngle().in(Degrees));
    SmartDashboard.putNumber(
        "Hood Stator Current",
        hoodFx.getStatorCurrent().getValue().in(Amps));
  }

  @Override
  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    hoodSim.setInputVoltage(hoodFx.getSimState().getMotorVoltage());
    hoodSim.update(0.020);

    double angleDeg = Math.max(
        MIN_ANGLE_DEG,
        Math.min(MAX_ANGLE_DEG, Math.toDegrees(hoodSim.getAngularPositionRad())));

    double rotorRotations =
        Degrees.of(angleDeg).in(edu.wpi.first.units.Units.Rotations) * GEAR_RATIO;

    hoodFx.getSimState().setRawRotorPosition(
        edu.wpi.first.units.Units.Rotations.of(rotorRotations));
    hoodFx.getSimState().setRotorVelocity(
        DegreesPerSecond.of(Math.toDegrees(hoodSim.getAngularVelocityRadPerSec()) * GEAR_RATIO));
  }
}
