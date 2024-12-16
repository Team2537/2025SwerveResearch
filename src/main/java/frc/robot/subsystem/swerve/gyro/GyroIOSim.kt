package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import java.util.function.Supplier

class GyroIOSim(private val speeds: Supplier<ChassisSpeeds>) : GyroIO {
    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        val robotSpeeds = speeds.get()
        inputs.connected = true
        inputs.yaw = inputs.yaw.plus(Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * 0.02))
        inputs.yawVelocity.mut_replace(robotSpeeds.omegaRadiansPerSecond / 0.02, Units.DegreesPerSecond)
    }
}
