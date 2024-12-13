package frc.robot.subsystem.swerve.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.subsystem.swerve.Drivebase
import lib.controllers.ControllerGains
import lib.math.units.angularToLinearD
import lib.math.units.angularToLinearV
import lib.math.units.into
import lib.math.units.linearToAngularV
import lib.math.units.measuredIn

class ModuleIOSim(
    private val driveMotor: DCMotor,
    private val turnMotor: DCMotor,
    private val driveGains: ControllerGains,
    private val turnGains: ControllerGains,
    private val gearings: Drivebase.Companion.ModuleGearings,
) : ModuleIO {
    private val driveMotorSim =
        DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotor, gearings.drive, 0.025), driveMotor)
    private val turnMotorSim =
        DCMotorSim(LinearSystemId.createDCMotorSystem(turnMotor, gearings.steer, 0.004), turnMotor)

    private val driveFeedback = PIDController(driveGains.kP, driveGains.kI, driveGains.kD)
    private val turnFeedback = PIDController(turnGains.kP, turnGains.kI, turnGains.kD)

    private val driveFeedforward = SimpleMotorFeedforward(driveGains.kS, 12 / 628.31853, driveGains.kA)

    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        driveMotorSim.update(0.02)
        turnMotorSim.update(0.02)

        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true
        inputs.absoluteEncoderConnected = true

        inputs.drivePosition.mut_replace(
            angularToLinearD(
                driveMotorSim.angularPosition,
                SwerveModule.wheelDiameter / (2.0),
            ),
        )
        inputs.driveVelocity.mut_replace(
            angularToLinearV(
                driveMotorSim.angularVelocity,
                SwerveModule.wheelDiameter / (2.0),
            ),
        )
        inputs.driveMotorVolts.mut_replace(driveMotorSim.inputVoltage, Volts)
        inputs.driveStatorCurrent.mut_replace(driveMotorSim.currentDrawAmps, Amps)

        inputs.turnPosition = Rotation2d.fromRadians(turnMotorSim.angularPositionRad)
        inputs.absoluteTurnPosition = inputs.turnPosition
        inputs.turnVelocity.mut_replace(turnMotorSim.angularVelocity)
        inputs.turnMotorVolts.mut_replace(turnMotorSim.inputVoltage, Volts)
        inputs.turnStatorCurrent.mut_replace(turnMotorSim.currentDrawAmps, Amps)
    }

    override fun runDriveVolts(volts: Voltage) {
        driveMotorSim.inputVoltage = volts into Volts
    }

    override fun runTurnVolts(volts: Voltage) {
        turnMotorSim.inputVoltage = volts into Volts
    }

    override fun runTurnPositionSetpoint(position: Rotation2d) {
//        runTurnVolts(
//            (
//                turnFeedback.calculate(
//                    turnMotorSim.angularPositionRotations,
//                    position.rotations,
//                ) measuredIn Volts
//            ) as Voltage,
//        )
        turnMotorSim.setState(position.radians, 0.0)
    }

    override fun runDriveVelocitySetpoint(velocityMetersPerSecond: Double) {
        val velocity = linearToAngularV(velocityMetersPerSecond, SwerveModule.wheelDiameter / (2.0))
        val feedforward = driveFeedforward.calculate(velocity)
        val output =
            (
                driveFeedback.calculate(
                    driveMotorSim.angularVelocityRPM,
                    velocity into RPM,
                ) measuredIn Volts
            ) as Voltage + feedforward
//        runDriveVolts(output)
        driveMotorSim.setState(driveMotorSim.angularPositionRad, velocity into RadiansPerSecond)
    }

    override fun stop() {
        driveMotorSim.inputVoltage = 0.0
        turnMotorSim.inputVoltage = 0.0
    }

    override fun reset() {
        driveMotorSim.setState(0.0, 0.0)
    }
}
