package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystem.swerve.Drivebase
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class AccelerationControlledDriveCommand(
    private val drivebase: Drivebase,
    private val xVel: DoubleSupplier,
    private val yVel: DoubleSupplier,
    private val thetaVel: DoubleSupplier,
    private val isFieldOriented: BooleanSupplier
) : Command() {
    private val cycleTime = 0.02
    private val maxForwardsAccel = 1000.0
    private val maxSidewaysAccel = 1000.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(drivebase)
    }

    override fun execute() {
        val desiredSpeeds: ChassisSpeeds =
            if(isFieldOriented.asBoolean){
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVel.asDouble,
                    yVel.asDouble,
                    thetaVel.asDouble,
                    drivebase.gyroInputs.yaw,
                )
            } else {
                ChassisSpeeds(
                    xVel.asDouble,
                    yVel.asDouble,
                    thetaVel.asDouble
                )
            }

        val currentSpeeds = drivebase.robotRelativeSpeeds

        var forwardsAccel = (desiredSpeeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond) / cycleTime
        var sidewaysAccel = (desiredSpeeds.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond) / cycleTime
        var angularAccel = (desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond) / cycleTime

        // Limit to stop the robot from tilting
        forwardsAccel = forwardsAccel.coerceIn(-maxForwardsAccel, maxForwardsAccel)
        sidewaysAccel = sidewaysAccel.coerceIn(-maxSidewaysAccel, maxSidewaysAccel)

        drivebase.applyChassisSpeeds(
            ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond + forwardsAccel * cycleTime,
                currentSpeeds.vyMetersPerSecond + sidewaysAccel * cycleTime,
                currentSpeeds.omegaRadiansPerSecond + angularAccel * cycleTime,
            ),
        )
    }
}
