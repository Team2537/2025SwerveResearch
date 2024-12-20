package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.MutAngularVelocity
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    class GyroInputs : LoggableInputs {
        var connected: Boolean = false
        var yaw: Rotation2d = Rotation2d()
        var yawVelocity: MutAngularVelocity = DegreesPerSecond.zero().mutableCopy()
        var pitch: Rotation2d = Rotation2d()
        var pitchVelocity: MutAngularVelocity = DegreesPerSecond.zero().mutableCopy()
        var roll: Rotation2d = Rotation2d()
        var rollVelocity: MutAngularVelocity = DegreesPerSecond.zero().mutableCopy()

        override fun toLog(table: LogTable) {
            table.put("connected", connected)
            table.put("yaw", Rotation2d.struct, yaw)
            table.put("yawVelocity", yawVelocity)
            table.put("pitch", Rotation2d.struct, pitch)
            table.put("pitchVelocity", pitchVelocity)
            table.put("roll", Rotation2d.struct, roll)
            table.put("rollVelocity", rollVelocity)
        }

        override fun fromLog(table: LogTable) {
            table.get("connected")?.let { connected = it.boolean }
            table.get("yaw", Rotation2d.struct, Rotation2d()).let { yaw = it }
            yawVelocity.mut_replace(table.get("yawVelocity", yawVelocity))
            table.get("pitch", Rotation2d.struct, Rotation2d()).let { pitch = it }
            pitchVelocity.mut_replace(table.get("pitchVelocity", pitchVelocity))
            table.get("roll", Rotation2d.struct, Rotation2d()).let { roll = it }
            rollVelocity.mut_replace(table.get("rollVelocity", rollVelocity))
        }
    }

    fun updateInputs(inputs: GyroInputs) {}

    fun setYaw(newYaw: Rotation2d) {}
}
