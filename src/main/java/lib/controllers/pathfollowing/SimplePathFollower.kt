package lib.controllers.pathfollowing

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import lib.controllers.ControllerGains
import java.util.function.Consumer
import java.util.function.Supplier

/**
 * Simple choreo path follower that uses the [SwerveSample]'s velocity feedforward, and three PID controllers for the
 * x, y, and theta components of the robot's pose.
 */
class SimplePathFollower(
    private val xFollowerGains: ControllerGains,
    private val yFollowerGains: ControllerGains,
    private val thetaFollowerGains: ControllerGains,
    private val speedConsumer: Consumer<ChassisSpeeds>,
    private val poseSupplier: Supplier<Pose2d>,
) : Consumer<SwerveSample> {
    private val xController = xFollowerGains.buildPIDController()
    private val yController = yFollowerGains.buildPIDController()
    private val thetaController = thetaFollowerGains.buildPIDController()

    override fun accept(sample: SwerveSample) {
        val pose = poseSupplier.get()

        val xOutput = xController.calculate(pose.x, sample.x)
        val yOutput = yController.calculate(pose.y, sample.y)
        val thetaOutput = thetaController.calculate(pose.rotation.radians, sample.heading)

        val speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xOutput + sample.vx,
                yOutput + sample.vy,
                thetaOutput + sample.omega,
                pose.rotation,
            )

        speedConsumer.accept(speeds)
    }
}
