package frc.robot.subsystem.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Swerve moduleConfig IO interface
 *
 * This interface defines the methods that a swerve moduleConfig must implement in order to be used in the
 * swerve drive
 */
interface ModuleIO {
    /**
     * The inputs for the moduleConfig
     *
     * This class is used to store the current state of the moduleConfig's sensors and motors
     */
    class ModuleInputs : LoggableInputs {
        var driveMotorConnected: Boolean = false
        var turnMotorConnected: Boolean = false
        var absoluteEncoderConnected: Boolean = false

        var drivePosition: MutDistance = Meters.zero().mutableCopy()
        var driveVelocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()
        var driveSupplyVolts: MutVoltage = Volts.zero().mutableCopy()
        var driveMotorVolts: MutVoltage = Volts.zero().mutableCopy()
        var driveStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var driveSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()

        var turnPosition: Rotation2d = Rotation2d()
        var absoluteTurnPosition: Rotation2d = Rotation2d()
        var turnVelocity: MutAngularVelocity = RadiansPerSecond.zero().mutableCopy()
        var turnSupplyVolts: MutVoltage = Volts.zero().mutableCopy()
        var turnMotorVolts: MutVoltage = Volts.zero().mutableCopy()
        var turnStatorCurrent: MutCurrent = Amps.zero().mutableCopy()
        var turnSupplyCurrent: MutCurrent = Amps.zero().mutableCopy()

        /** @suppress */
        override fun toLog(table: LogTable) {
            table.put("driveMotorConnected", driveMotorConnected)
            table.put("turnMotorConnected", turnMotorConnected)
            table.put("absoluteEncoderConnected", absoluteEncoderConnected)

            table.put("drivePosition", drivePosition)
            table.put("driveVelocity", driveVelocity)
            table.put("driveSupplyVolts", driveSupplyVolts)
            table.put("driveMotorVolts", driveMotorVolts)
            table.put("driveStatorCurrent", driveStatorCurrent)
            table.put("driveSupplyCurrent", driveSupplyCurrent)

            table.put("turnPosition", Rotation2d.struct, turnPosition)
            table.put("absoluteTurnPosition", Rotation2d.struct, absoluteTurnPosition)
            table.put("turnVelocity", turnVelocity)
            table.put("turnSupplyVolts", turnSupplyVolts)
            table.put("turnMotorVolts", turnMotorVolts)
            table.put("turnStatorCurrent", turnStatorCurrent)
            table.put("turnSupplyCurrent", turnSupplyCurrent)
        }

        /** @suppress */
        override fun fromLog(table: LogTable) {
            table.get("driveMotorConnected")?.let { driveMotorConnected = it.boolean }
            table.get("turnMotorConnected")?.let { turnMotorConnected = it.boolean }
            table.get("absoluteEncoderConnected")?.let { absoluteEncoderConnected = it.boolean }

            drivePosition.mut_replace(table.get("drivePosition", drivePosition))
            driveVelocity.mut_replace(table.get("driveVelocity", driveVelocity))
            driveSupplyVolts.mut_replace(table.get("driveSupplyVolts", driveSupplyVolts))
            driveMotorVolts.mut_replace(table.get("driveMotorVolts", driveMotorVolts))
            driveStatorCurrent.mut_replace(table.get("driveStatorCurrent", driveStatorCurrent))
            driveSupplyCurrent.mut_replace(table.get("driveSupplyCurrent", driveSupplyCurrent))

            table.get("turnPosition", Rotation2d.struct, Rotation2d())?.let { turnPosition = it }
            table.get("absoluteTurnPosition", Rotation2d.struct, Rotation2d())?.let {
                absoluteTurnPosition = it
            }
            turnVelocity.mut_replace(table.get("turnVelocity", turnVelocity))
            turnSupplyVolts.mut_replace(table.get("turnSupplyVolts", turnSupplyVolts))
            turnMotorVolts.mut_replace(table.get("turnMotorVolts", turnMotorVolts))
            turnStatorCurrent.mut_replace(table.get("turnStatorCurrent", turnStatorCurrent))
            turnSupplyCurrent.mut_replace(table.get("turnSupplyCurrent", turnSupplyCurrent))
        }
    }

    /**
     * Update the inputs using the current state of the moduleConfig
     *
     * @param inputs The inputs to update (mutated in place)
     */
    fun updateInputs(inputs: ModuleInputs) {}

    /**
     * Run the drive motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    fun runDriveVolts(volts: Voltage) {}

    /**
     * Run the turn motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    fun runTurnVolts(volts: Voltage) {}

    /**
     * Set the position setpoint for the turn motor
     *
     * @param position The position to set the motor to in radians
     */
    fun runTurnPositionSetpoint(position: Rotation2d) {}

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityMetersPerSecond The velocity to set the motor to in radians per second
     */
    fun runDriveVelocitySetpoint(velocityMetersPerSecond: Double) {}

    /** Stop the moduleConfig */
    fun stop() {}

    /** Reset the moduleConfig's position, for odometry purposes */
    fun reset() {}
}
