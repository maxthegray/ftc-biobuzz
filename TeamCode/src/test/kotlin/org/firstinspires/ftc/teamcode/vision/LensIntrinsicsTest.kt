package org.firstinspires.ftc.teamcode.vision

import java.io.File
import javax.xml.parsers.DocumentBuilderFactory
import kotlin.math.atan
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.w3c.dom.Element

class LensIntrinsicsTest {

    /** goBILDA 3122-0004-0001 at 640×480, from the user guide table. */
    private val gobilda640 = LensIntrinsics(
        widthPx = 640, heightPx = 480, fx = 545.584, fy = 545.584, cx = 350.588, cy = 223.150,
        distortion = listOf(0.0290195, -0.0238128, 0.0, 0.0, -0.0378927, 0.0, 0.0, 0.0),
    )

    @Test
    fun principalPointIsZeroAngleNotImageCentre() {
        val (h, v) = gobilda640.rayAnglesDegrees(350.588, 223.150)
        assertEquals(0.0, h, 1e-9)
        assertEquals(0.0, v, 1e-9)
        val (hCentre, _) = gobilda640.rayAnglesDegrees(320.0, 240.0)
        assertTrue("image centre is left of the principal point", hCentre < -3.0)
    }

    @Test
    fun signsArePositiveRightAndPositiveDown() {
        val (right, _) = gobilda640.rayAnglesDegrees(600.0, 223.15)
        val (_, down) = gobilda640.rayAnglesDegrees(350.588, 450.0)
        assertTrue(right > 0.0)
        assertTrue(down > 0.0)
    }

    @Test
    fun undistortionInvertsTheForwardModelAcrossTheFrame() {
        val lens = gobilda640.copy(distortion = listOf(0.0290195, -0.0238128, 0.001, -0.0005, -0.0378927, 0.0, 0.0, 0.0))
        for (x in listOf(0.0, 100.0, 320.0, 639.0)) {
            for (y in listOf(0.0, 240.0, 479.0)) {
                val (xn, yn) = lens.normalizedUndistorted(x, y)
                val (xp, yp) = lens.distortToPixel(xn, yn)
                assertEquals(x, xp, 1e-3)
                assertEquals(y, yp, 1e-3)
            }
        }
    }

    @Test
    fun withoutDistortionAnglesArePlainPinhole() {
        val pinhole = gobilda640.copy(distortion = List(8) { 0.0 })
        val (h, _) = pinhole.rayAnglesDegrees(350.588 + 545.584, 223.15)
        assertEquals(45.0, h, 1e-9)
        assertEquals(Math.toDegrees(atan(100 / 545.584)), pinhole.rayAnglesDegrees(450.588, 0.0).first, 1e-9)
    }

    @Test
    fun teamCalibrationFileCarriesGobildaGuideValuesAndKeepsExistingEntries() {
        val file = listOf(
            File("src/main/res/xml/teamwebcamcalibrations.xml"),
            File("TeamCode/src/main/res/xml/teamwebcamcalibrations.xml"),
        ).first { it.exists() }
        val text = file.readText()
        assertTrue("SDK example comments preserved", text.contains("Logitech HD Pro Webcam C920"))

        val doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file)
        val cameras = doc.getElementsByTagName("Camera")
        val gobilda = (0 until cameras.length).map { cameras.item(it) as Element }
            .single { Integer.decode(it.getAttribute("vid")) == 0x0C45 && Integer.decode(it.getAttribute("pid")) == 0x0366 }
        val calibrations = gobilda.getElementsByTagName("Calibration")
        val bySize = (0 until calibrations.length).map { calibrations.item(it) as Element }
            .associateBy { it.getAttribute("size").replace(",", " ").trim().split(Regex("\\s+")).joinToString("x") }

        assertEquals(setOf("1280x800", "1280x720", "800x600", "640x480", "320x240"), bySize.keys)
        assertEquals(listOf(545.584, 545.584), floats(bySize.getValue("640x480").getAttribute("focalLength")))
        assertEquals(listOf(350.588, 223.150), floats(bySize.getValue("640x480").getAttribute("principalPoint")))
        assertEquals(listOf(270.260, 270.260), floats(bySize.getValue("320x240").getAttribute("focalLength")))
        assertEquals(listOf(908.683, 908.683), floats(bySize.getValue("1280x720").getAttribute("focalLength")))
        for (entry in bySize.values) assertEquals(8, floats(entry.getAttribute("distortionCoefficients")).size)
    }

    private fun floats(attribute: String): List<Double> =
        attribute.replace(",", " ").trim().split(Regex("\\s+")).map { it.removeSuffix("f").toDouble() }
}
