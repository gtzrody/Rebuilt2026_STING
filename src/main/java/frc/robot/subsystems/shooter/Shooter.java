package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.MechanismConstants;


public class Shooter extends SubsystemBase {
  private static final int LEFT_ID = 13;
  private static final int RIGHT_ID = 14;
  private static final double GEAR_RATIO =
      36.0 / 25.0; 

  private static final double KP = 0.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KS = 0.35;
  private static final double KV = 0.167;
  private static final double KA = 0.0;

  private static final double MAX_RPM = 4900.0;

  private static final double WHEEL_DIAMETER_METERS = Inches.of(4).in(edu.wpi.first.units.Units.Meters);
  private static final double WHEEL_MASS_KG = Pounds.of(1.54).in(edu.wpi.first.units.Units.Kilograms);
  private static final double WHEEL_MOI =
      0.5 * WHEEL_MASS_KG * Math.pow(WHEEL_DIAMETER_METERS / 2.0, 2);

  private final TalonFX leftTalon = new TalonFX(LEFT_ID, new CANBus("canivore"));
  private final TalonFX rightTalon = new TalonFX(RIGHT_ID, new CANBus("canivore"));

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  private final DCMotorSim rightSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), WHEEL_MOI, GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  private final DCMotorSim leftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), WHEEL_MOI, GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  public Shooter() {
    configureMotor(rightTalon, true);
    configureMotor(leftTalon, false);

    leftTalon.setControl(new Follower(RIGHT_ID, MotorAlignmentValue.Opposed));
  }

  private void configureMotor(TalonFX motor, boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted =
        inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Slot0.kP = KP;
    config.Slot0.kI = KI;
    config.Slot0.kD = KD;
    config.Slot0.kS = KS;
    config.Slot0.kV = KV;
    config.Slot0.kA = KA;

    motor.getConfigurator().apply(config);
  }


  public AngularVelocity getVelocity() {
    return rightTalon.getVelocity().getValue();
  }

  /**
   * Returns a command that continuously holds the requested wheel speed.
   */
  public Command setVelocity(AngularVelocity speed) {
    return run(() -> setVelocitySetpoint(speed));
  }

  /**
   * Sets the persistent velocity setpoint.
   */
  public void setVelocitySetpoint(AngularVelocity speed) {
    double rpm = speed.in(RPM);
    double clampedRpm = Math.max(-MAX_RPM, Math.min(MAX_RPM, rpm));
    velocityRequest.withVelocity(RPM.of(clampedRpm));
    rightTalon.setControl(velocityRequest);
  }

  /**
   * Manual duty-cycle control, used by the default command and feed command.
   */
  public Command set(double dutyCycle) {
    return run(() -> setDutyCycle(dutyCycle));
  }

  private void setDutyCycle(double dutyCycle) {
    double output = Math.max(-1.0, Math.min(1.0, dutyCycle));
    dutyCycleRequest.withOutput(output);
    rightTalon.setControl(dutyCycleRequest);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getVelocity().in(RPM));
    SmartDashboard.putNumber("Shooter Setpoint RPM", velocityRequest.getVelocityMeasure().in(RPM));
    SmartDashboard.putNumber(
        "Shooter Supply Current", rightTalon.getSupplyCurrent().getValue().in(Amps));
  }

  @Override
  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    double batteryVoltage = edu.wpi.first.wpilibj.simulation.RoboRioSim.getVInVoltage();
    rightTalon.getSimState().setSupplyVoltage(batteryVoltage);
    leftTalon.getSimState().setSupplyVoltage(batteryVoltage);

    rightSim.setInputVoltage(rightTalon.getSimState().getMotorVoltage());
    leftSim.setInputVoltage(leftTalon.getSimState().getMotorVoltage());

    rightSim.update(0.020);
    leftSim.update(0.020);

    rightTalon.getSimState().setRawRotorPosition(
        edu.wpi.first.units.Units.Radians.of(rightSim.getAngularPositionRad() * GEAR_RATIO));
    rightTalon.getSimState().setRotorVelocity(
        edu.wpi.first.units.Units.RadiansPerSecond.of(
            rightSim.getAngularVelocityRadPerSec() * GEAR_RATIO));

  }
}
