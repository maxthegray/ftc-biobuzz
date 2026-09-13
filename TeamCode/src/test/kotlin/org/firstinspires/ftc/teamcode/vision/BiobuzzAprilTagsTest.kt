package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.CellLocation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class BiobuzzAprilTagsTest {

    @Test
    fun catalogMatchesCompetitionManualSection9_9() {
        assertEquals((30..45).toList(), BiobuzzAprilTags.tags.map { it.id })
        assertEquals("36h11", BiobuzzAprilTags.FAMILY)
        assertEquals(82.55, BiobuzzAprilTags.TAG_SIZE_MM, 1e-9)

        assertCell(30..33, Alliance.RED, CellLocation.FAR, "RED SCORING")
        assertCell(34..37, Alliance.RED, CellLocation.AUDIENCE, "RED AUDIENCE")
        assertCell(38..41, Alliance.BLUE, CellLocation.AUDIENCE, "BLUE AUDIENCE")
        assertCell(42..45, Alliance.BLUE, CellLocation.FAR, "BLUE SCORING")
    }

    @Test
    fun slotsFollowPrintedOrderAndFigure9_15Offsets() {
        val cell = BiobuzzAprilTags.lookup(38)!!.cell
        val tags = BiobuzzAprilTags.tagsOf(cell)
        assertEquals(listOf(1, 2, 3, 4), tags.map { it.slot })
        assertEquals(listOf(-6.5, -2.75, 2.75, 6.5), tags.map { it.offsetFromClusterCenterInches })
        assertEquals("BLUE AUDIENCE slot 3", BiobuzzAprilTags.lookup(40)!!.meaning)
    }

    @Test
    fun unknownIdsAreNotSeasonTags() {
        assertNull(BiobuzzAprilTags.lookup(29))
        assertNull(BiobuzzAprilTags.lookup(46))
        assertNull(BiobuzzAprilTags.lookup(0))
    }

    @Test
    fun limelightFamilySpellingsAreRecognized() {
        assertTrue(BiobuzzAprilTags.isSeasonFamily("36H11C"))
        assertTrue(BiobuzzAprilTags.isSeasonFamily("36h11"))
        assertTrue(BiobuzzAprilTags.isSeasonFamily("tag36h11"))
        assertFalse(BiobuzzAprilTags.isSeasonFamily("16H5C"))
    }

    private fun assertCell(ids: IntRange, alliance: Alliance, location: CellLocation, sticker: String) {
        val tags = ids.map { BiobuzzAprilTags.lookup(it)!! }
        assertTrue(tags.all { it.cell.alliance == alliance && it.cell.location == location })
        assertEquals(sticker, tags.first().cell.stickerLabel)
        assertEquals(tags.map { it.cell }.toSet(), setOf(BiobuzzAprilTags.Cell(alliance, location)))
    }
}
