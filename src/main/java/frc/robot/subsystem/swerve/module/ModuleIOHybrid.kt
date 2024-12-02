package frc.robot.subsystem.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import frc.robot.subsystem.swerve.Drivebase
import lib.controllers.ControllerGains
import lib.math.units.into
import kotlin.math.PI

/**
 * IO implementation for the Kraken Drive and SparkMax steer (KDSMS) swerve module.
 *
 * @see ModuleIO
 */
class ModuleIOHybrid(
    val moduleConfig: Drivebase.Companion.ModuleConfig,
    val gearings: Drivebase.Companion.ModuleGearings,
    val driveGains: ControllerGains,
    val turnGains: ControllerGains,
) : ModuleIO {
    val driveMotor =
        TalonFX(moduleConfig.driveID).also {
            val config = TalonFXConfiguration()

            config.MotorOutput.apply {
                Inverted = if (moduleConfig.driveInverted) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }

            config.Slot0 =
                Slot0Configs().apply {
                    kP = driveGains.kP
                    kI = driveGains.kI
                    kD = driveGains.kD
                    kS = driveGains.kS
                    kV = driveGains.kV
                    kA = driveGains.kA
                }

            config.Feedback.apply { SensorToMechanismRatio = gearings.drive }

            config.CurrentLimits.apply {
                withStatorCurrentLimit(SwerveModule.slipCurrent)
                StatorCurrentLimitEnable = true
            }

            config.TorqueCurrent.PeakForwardTorqueCurrent = (SwerveModule.slipCurrent into Amps)
            config.TorqueCurrent.PeakReverseTorqueCurrent = -(SwerveModule.slipCurrent into Amps)

            it.configurator.apply(config)
        }

    val drivePosition = driveMotor.position.clone()
    val driveVelocity = driveMotor.velocity.clone()
    val driveSupplyVolts = driveMotor.supplyVoltage.clone()
    val driveMotorVolts = driveMotor.motorVoltage.clone()
    val driveStatorCurrent = driveMotor.statorCurrent.clone()
    val driveSupplyCurrent = driveMotor.supplyCurrent.clone()

    val turnMotor: SparkMax =
        SparkMax(moduleConfig.turnID, MotorType.kBrushless).also {
            val config =
                SparkMaxConfig()
                    .inverted(moduleConfig.turnInverted)
                    .idleMode(SparkBaseConfig.IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .apply(ClosedLoopConfig().pid(turnGains.kP, turnGains.kI, turnGains.kD))
                    .apply(
                        EncoderConfig()
                            .positionConversionFactor(1 / gearings.steer)
                            .velocityConversionFactor(1 / gearings.steer),
                    )

            it.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters,
            )
        }

    val absoluteEncoder: CANcoder =
        CANcoder(moduleConfig.encoderID).also {
            val config = CANcoderConfiguration()

            config.MagnetSensor.MagnetOffset = moduleConfig.encoderOffset

            it.configurator.apply(config)
        }

    val absoluteTurnPosition = absoluteEncoder.absolutePosition.clone()

    val driveOpenLoopRequest: VoltageOut = VoltageOut(0.0)
    val driveClosedLoopRequest: VelocityVoltage = VelocityVoltage(0.0)

    init {
        drivePosition.setUpdateFrequency(100.0)
        driveVelocity.setUpdateFrequency(100.0)
        driveSupplyVolts.setUpdateFrequency(100.0)
        driveMotorVolts.setUpdateFrequency(100.0)
        driveStatorCurrent.setUpdateFrequency(100.0)
        driveSupplyCurrent.setUpdateFrequency(100.0)

        absoluteTurnPosition.setUpdateFrequency(100.0)

        driveMotor.optimizeBusUtilization()
        absoluteEncoder.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.driveMotorConnected =
            BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveSupplyVolts,
                driveMotorVolts,
                driveStatorCurrent,
                driveSupplyCurrent,
            )
                .isOK

        inputs.turnMotorConnected = true

        inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(absoluteTurnPosition).isOK

        inputs.drivePosition.mut_replace(
            2 * PI * drivePosition.valueAsDouble * ((SwerveModule.wheelDiameter into Meters) / 2),
            Meters,
        )
        inputs.driveVelocity.mut_replace(
            2 * PI * driveVelocity.valueAsDouble * ((SwerveModule.wheelDiameter into Meters) / 2),
            MetersPerSecond,
        )

        inputs.driveSupplyVolts.mut_replace(driveSupplyVolts.value)
        inputs.driveMotorVolts.mut_replace(driveMotorVolts.value)
        inputs.driveStatorCurrent.mut_replace(driveStatorCurrent.value)
        inputs.driveSupplyCurrent.mut_replace(driveSupplyCurrent.value)

        inputs.turnPosition = Rotation2d.fromRotations(turnMotor.encoder.position)
        inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.valueAsDouble)

        inputs.turnVelocity.mut_replace(turnMotor.encoder.velocity, RotationsPerSecond)
        inputs.turnSupplyVolts.mut_replace(turnMotor.busVoltage, Volts)
        inputs.turnMotorVolts.mut_replace(turnMotor.appliedOutput * turnMotor.busVoltage, Volts)
        inputs.turnStatorCurrent.mut_replace(turnMotor.outputCurrent, Amps)
    }

    override fun runDriveVolts(volts: Voltage) {
        driveMotor.setControl(driveOpenLoopRequest.withOutput(volts))
    }

    override fun runTurnVolts(volts: Voltage) {
        turnMotor.setVoltage(volts into Volts)
    }

    override fun runTurnPositionSetpoint(position: Rotation2d) {
        turnMotor.closedLoopController.setReference(
            position.rotations,
            SparkBase.ControlType.kPosition,
            0,
            turnGains.kS,
        )
    }

    override fun runDriveVelocitySetpoint(velocityMetersPerSecond: Double) {
        driveMotor.setControl(
            driveClosedLoopRequest.withVelocity(
                velocityMetersPerSecond / (2 * PI * ((SwerveModule.wheelDiameter into Meters) / 2)),
            ),
        )
    }

    override fun stop() {
        driveMotor.stopMotor()
        turnMotor.stopMotor()
    }

    override fun reset() {
        driveMotor.setPosition(0.0)
    }
}
