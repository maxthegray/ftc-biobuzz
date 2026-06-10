package org.firstinspires.ftc.teamcode.core.logging

import org.junit.Assert.assertEquals
import org.junit.Test

class SchedulerIntrospectionTest {

    @Test
    fun reflectionFailureDisablesFutureReads() {
        val introspection = SchedulerIntrospection(String::class.java)

        assertEquals(emptyList<String>(), introspection.runningCommandNames())
        assertEquals(emptyList<String>(), introspection.runningCommandNames())
        assertEquals(0, introspection.activeRequirementCount())
    }
}
