package org.firstinspires.ftc.teamcode.starter.localization

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Utility helpers that reach down to the raw [GoBildaPinpointDriver] in the
 * hardware map for things Pedro's wrapper does not expose — specifically
 * the IMU recalibration sequence that takes the Pinpoint ~250 ms to run
 * and should only happen once at init while the robot is physically still.
 *
 * The `PinpointLocalizer` Pedro uses is still constructed via
 * [Constants.createFollower] — this file is NOT a replacement, it is a
 * companion that you call by hand when you need to reset the IMU or read
 * raw device status for diagnostics.
 */
object PinpointLocalizer {

    /**
     * Looks up the raw Pinpoint driver using the same hardware name declared
     * in [Constants.localizerConstants]. Throws if the device is absent
     * from the configuration.
     */
    fun raw(hardwareMap: HardwareMap): GoBildaPinpointDriver {
        val name = Constants.pinpointHardwareName
        return hardwareMap.get(GoBildaPinpointDriver::class.java, name)
    }

    /**
     * Recalibrate the Pinpoint's internal IMU and zero its pose. The robot
     * MUST be physically still for ~250 ms while this runs; call it during
     * init, not during the match.
     *
     * After recalibration the follower's starting pose is reapplied so
     * Pedro's cached origin doesn't drift.
     */
    fun recalibrate(hardwareMap: HardwareMap, follower: Follower, resetPose: Pose = Pose()) {
        val odo = raw(hardwareMap)
        odo.recalibrateIMU()
        odo.resetPosAndIMU()
        // Give the Pinpoint its ~250 ms calibration window.
        Thread.sleep(300)
        follower.setStartingPose(resetPose)
    }

    /**
     * Quick health check — returns the driver's self-reported device status
     * for surfacing in telemetry ("READY", "NOT_READY", "FAULT_*", etc.).
     */
    fun status(hardwareMap: HardwareMap): GoBildaPinpointDriver.DeviceStatus =
        raw(hardwareMap).deviceStatus
}
