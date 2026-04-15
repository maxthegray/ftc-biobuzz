package org.firstinspires.ftc.teamcode.general.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Puts every connected Lynx hub (Control Hub + any Expansion Hubs) into
 * MANUAL bulk-read mode and centralises cache clearing.
 *
 * Why: the default AUTO mode issues a fresh bulk read on *every* getPower,
 * getPosition, or getVoltage call. Reading eight encoders and a voltage
 * sensor from a single command could cost ~12–20 ms. In MANUAL mode each
 * bulk read fills the cache once, and every subsequent getter this tick is
 * free — typical savings are 5–8 ms per loop.
 *
 * The rule: call [clearCaches] exactly once at the top of the main loop
 * (before subsystems' `periodic`). Never read any motor/encoder value
 * outside of that window unless you also call [clearCaches] first.
 */
class BulkReadManager(private val hardwareMap: HardwareMap) {

    private var modules: List<LynxModule> = emptyList()

    fun init() {
        modules = hardwareMap.getAll(LynxModule::class.java)
        for (m in modules) m.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

    /** Drop every module's cached bulk data so the next getter forces a fresh read. */
    fun clearCaches() {
        for (m in modules) m.clearBulkCache()
    }

    /** Returns modules discovered at [init]. Mostly useful for telemetry. */
    fun modules(): List<LynxModule> = modules
}
