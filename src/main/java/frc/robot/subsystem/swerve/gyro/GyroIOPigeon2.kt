package frc.robot.subsystems.swerve.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Degrees
import lib.math.units.into

class GyroIOPigeon2(private val canBusName: String = "", private val id: Int) : GyroIO {
    private val gyro: Pigeon2 = Pigeon2(id, canBusName)

    private val yawGetter = gyro.yaw
    private val yawRateGetter = gyro.angularVelocityZWorld
    private val pitchGetter = gyro.pitch
    private val pitchRateGetter = gyro.angularVelocityXWorld
    private val rollGetter = gyro.roll
    private val rollRateGetter = gyro.angularVelocityYWorld

    init {
        val gyroConfigs: Pigeon2Configuration = Pigeon2Configuration()
        gyro.configurator.setYaw(0.0)
        gyro.configurator.apply(gyroConfigs)
        yawGetter.setUpdateFrequency(100.0)
        yawRateGetter.setUpdateFrequency(100.0)
        pitchGetter.setUpdateFrequency(100.0)
        pitchRateGetter.setUpdateFrequency(100.0)
        rollGetter.setUpdateFrequency(100.0)
        rollRateGetter.setUpdateFrequency(100.0)
        gyro.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                yawGetter,
                yawRateGetter,
            ).isOK

        inputs.yaw = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yawGetter, yawRateGetter) into Degrees)
        inputs.yawVelocity.mut_replace(yawRateGetter.value)
        inputs.pitch = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(pitchGetter, pitchRateGetter) into Degrees)
        inputs.pitchVelocity.mut_replace(pitchRateGetter.value)
        inputs.roll = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(rollGetter, rollRateGetter) into Degrees)
        inputs.rollVelocity.mut_replace(rollRateGetter.value)
    }

    override fun setYaw(newYaw: Rotation2d) {
        gyro.setYaw(newYaw.degrees)
    }
}
