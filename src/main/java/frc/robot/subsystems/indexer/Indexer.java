package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {
  private static final int MOTOR_ID = 18;
  private static final String CAN_BUS = "canivore";
  private static final double GEAR_RATIO = 36.0 / 25.0;

  private final TalonFX indexerMotor = new TalonFX(MOTOR_ID, new CANBus(CAN_BUS));
  private final DutyCycleOut outputRequest = new DutyCycleOut(0);

  private final DCMotorSim simulation = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1),
          0.001,
          GEAR_RATIO),
      DCMotor.getKrakenX60(1));

  public Indexer() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 90.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

    indexerMotor.getConfigurator().apply(config);
  }

  /**
   * Run the indexer.
   *
   * @param output motor output from -1 to +1
   */
  public Command set(double output) {
    return run(() -> {
      double clamped = Math.max(-1.0, Math.min(1.0, output));
      indexerMotor.setControl(outputRequest.withOutput(clamped));
    });
  }

  /** Stop the indexer immediately. */
  public void stop() {
    indexerMotor.stopMotor();
  }

  public double getOutput() {
    return indexerMotor.getDutyCycle().getValueAsDouble();
  }

  public double getRollerRPM() {
    return indexerMotor.getVelocity().getValue().in(Units.RPM) / GEAR_RATIO;
  }

  @Override
  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    simulation.setInputVoltage(indexerMotor.getSimState().getMotorVoltage());
    simulation.update(0.020);
  }
}
