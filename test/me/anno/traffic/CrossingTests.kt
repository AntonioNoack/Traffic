package me.anno.traffic

import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs

class CrossingTests {

    class TestCrossing(center: Vector3d, radius: Double, initialAllowed: Int = 0) : Crossing(center, radius) {
        var allowedSection: Int = initialAllowed

        override fun mayDrive(from: Int, to: Int): Boolean {
            return from == allowedSection
        }
    }

    private fun createLanePoint(x: Double, z: Double, angle: Double): LanePoint {
        return LanePoint(Vector3d(x, 0.0, z), Quaternionf().rotationY(angle.toFloat()), angle, 2.0)
    }

    private fun createLane(start: LanePoint, mid: LanePoint, end: LanePoint): Lane {
        return Lane(start, mid, end)
    }

    private fun extendLane(from: LanePoint, to: LanePoint): Lane {
        val control = LanePoint(
            Vector3d(
                (from.position.x + to.position.x) * 0.5,
                0.0,
                (from.position.z + to.position.z) * 0.5
            ),
            Quaternionf().rotationY(((from.angle + to.angle) * 0.5).toFloat()),
            (from.angle + to.angle) * 0.5,
            2.0
        )
        return Lane(from, control, to)
    }

    @Test
    fun testSimpleIntersectionNorthSouthOpen() {
        // 4-way intersection with 2 lanes each direction
        // N->S = 0, E->W = 1, S->N = 2, W->E = 3
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 0)

        val northToCenter = createLanePoint(0.0, -100.0, PI * 0.5)
        val northToCrossing = createLanePoint(0.0, -15.0, PI * 0.5)
        val southFromCrossing = createLanePoint(0.0, 15.0, PI * 0.5)
        val southToEnd = createLanePoint(0.0, 100.0, PI * 0.5)

        // Lane going North to South
        val laneNS = createLane(northToCenter, northToCrossing, southFromCrossing)
        val laneNS2 = extendLane(southFromCrossing, southToEnd)

        // Create crossing sections
        val cs0 = CrossingSection(crossing, 0)
        val cs1 = CrossingSection(crossing, 1)
        val cs2 = CrossingSection(crossing, 2)
        val cs3 = CrossingSection(crossing, 3)

        laneNS.crossingSection = cs0
        laneNS2.crossingSection = cs1

        crossing.sections.add(cs0)
        crossing.sections.add(cs1)
        crossing.sections.add(cs2)
        crossing.sections.add(cs3)

        // N->S direction should be allowed
        assertTrue(crossing.mayDrive(0, 1))
        assertFalse(crossing.mayDrive(1, 2))
        assertFalse(crossing.mayDrive(2, 3))
        assertFalse(crossing.mayDrive(3, 0))
    }

    @Test
    fun testVehicleStopsAtRedLight() {
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 1)

        val lanePoints = createFourWayLanes(crossing)
        for (cs in lanePoints.keys) {
            lanePoints[cs]!!.forEach { it.crossingSection = cs }
        }

        crossing.sections.addAll(lanePoints.keys)

        val northToSouthLane = lanePoints[CrossingSection(crossing, 0)]!!

        val vehicle = Vehicle()
        vehicle.route.addAll(northToSouthLane)
        vehicle.position.set(0.0, 0.0, -20.0)
        vehicle.rotation.rotationY((PI * 0.5).toFloat())
        vehicle.maxVelocity = 13.0

        val initialSpeed = vehicle.velocity.length()
        assertTrue(initialSpeed < 0.1, "Vehicle should start stopped")

        vehicle.update(0.1f)
        val afterUpdate = vehicle.velocity.length()

        // Should not have accelerated because N->S is NOT allowed (allowedSection = 1)
        assertTrue(afterUpdate < 0.5, "Vehicle should not accelerate when light is red")
    }

    @Test
    fun testVehicleProceedsOnGreenLight() {
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 0)

        val lanePoints = createFourWayLanes(crossing)
        for (cs in lanePoints.keys) {
            lanePoints[cs]!!.forEach { it.crossingSection = cs }
        }

        crossing.sections.addAll(lanePoints.keys)

        val northToSouthLane = lanePoints[CrossingSection(crossing, 0)]!!

        val vehicle = Vehicle()
        vehicle.route.addAll(northToSouthLane)
        vehicle.position.set(0.0, 0.0, -20.0)
        vehicle.rotation.rotationY((PI * 0.5).toFloat())
        vehicle.maxVelocity = 13.0
        vehicle.velocity.set(0.0, 0.0, 5.0)

        vehicle.update(0.1f)

        assertTrue(vehicle.velocity.z > 0, "Vehicle should proceed when light is green")
    }

    @Test
    fun testTwoLanePerDirectionNoTurnSideways() {
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 0)

        val lanePoints = createFourWayLanes(crossing)
        for (cs in lanePoints.keys) {
            lanePoints[cs]!!.forEach { it.crossingSection = cs }
        }

        crossing.sections.addAll(lanePoints.keys)

        val northToSouthLane = lanePoints[CrossingSection(crossing, 0)]!!

        val vehicle = Vehicle()
        vehicle.route.addAll(northToSouthLane)
        vehicle.position.set(0.0, 0.0, -50.0)
        vehicle.rotation.rotationY((PI * 0.5).toFloat())
        vehicle.maxVelocity = 13.0
        vehicle.velocity.set(0.0, 0.0, 10.0)

        vehicle.update(0.1f)

        // Forward direction should still be +Z (going north to south)
        val forward = vehicle.rotation.transform(Vector3d(0.0, 0.0, 1.0))

        // Should not have turned sideways (90 degrees would be X axis)
        assertTrue(abs(forward.z) > abs(forward.x),
            "Vehicle should not turn sideways, forward.z=${forward.z}, forward.x=${forward.x}")
    }

    @Test
    fun testSwitchingTrafficPhase() {
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 0)

        assertEquals(0, crossing.allowedSection)

        crossing.allowedSection = 1

        assertFalse(crossing.mayDrive(0, 1))
        assertTrue(crossing.mayDrive(1, 2))
    }

    @Test
    fun testVehicleCrossesIntersectionLikeUITest() {
        val crossing = object : Crossing(Vector3d(0.0, 0.0, 0.0), 15.0) {
            override fun mayDrive(from: Int, to: Int): Boolean = true
        }

        // Build the same kind of turn geometry UITest.kt creates:
        // entry lane -> in-crossing connector -> exit lane.
        val entry0 = createLanePoint(0.0, -100.0, PI * 0.5)
        val entry1 = createLanePoint(0.0, -15.0, PI * 0.5)
        val exit0 = createLanePoint(15.0, 0.0, 0.0)
        val exit1 = createLanePoint(100.0, 0.0, 0.0)

        val entryLane = createLane(entry0, createLanePoint(0.0, -50.0, PI * 0.5), entry1)

        val entryDir = Vector3d(entryLane.getPosition(1.01, 0.0, 0.0, Vector3d()))
            .sub(entry1.position)
            .normalize()
            .mul(10.0)
        val exitLane = createLane(exit0, createLanePoint(50.0, 0.0, 0.0), exit1)
        val exitDir = Vector3d(exitLane.getPosition(-0.01, 0.0, 0.0, Vector3d()))
            .sub(exit0.position)
            .normalize()
            .mul(10.0)

        val centerPoint = Vector3d(entry1.position).add(entryDir).mix(Vector3d(exit0.position).add(exitDir), 0.5)
        val angle = (entry1.angle + exit0.angle) * 0.5
        val turnLane = Lane(
            entry1,
            LanePoint(centerPoint, Quaternionf().rotationY(angle.toFloat()), angle, 2.0),
            exit0
        )

        val csEntry = CrossingSection(crossing, 1)
        val csTurn = CrossingSection(crossing, 0)
        val csExit = CrossingSection(crossing, 2)
        entryLane.crossingSection = csEntry
        turnLane.crossingSection = csTurn
        exitLane.crossingSection = csExit
        crossing.sections.addAll(listOf(csEntry, csTurn, csExit))

        val vehicle = Vehicle()
        vehicle.route.add(entryLane)
        vehicle.route.add(turnLane)
        vehicle.route.add(exitLane)
        vehicle.position.set(0.0, 0.0, -60.0)
        vehicle.rotation.rotationY((PI * 0.5).toFloat())
        vehicle.velocity.set(0.0, 0.0, 8.0)
        vehicle.maxVelocity = 13.0

        val dt = 0.05f
        for (i in 0 until 400) {
            vehicle.update(dt)
        }

        val forward = vehicle.rotation.transform(Vector3d(0.0, 0.0, 1.0))
        assertFalse(vehicle.isCrashed)
        assertTrue(vehicle.routeIndex >= 2, "Vehicle should have reached the exit lane")
        assertTrue(vehicle.position.x > 20.0, "Vehicle should have crossed the intersection")
        assertTrue(abs(forward.x) > abs(forward.z),
            "Vehicle should be heading east after the turn, forward=$forward")
    }

    @Test
    fun testVehiclesDoNotPassThroughAtCrossing() {
        val crossing = TestCrossing(Vector3d(0.0, 0.0, 0.0), 15.0, initialAllowed = 0)

        val lanePoints = createFourWayLanes(crossing)
        for (cs in lanePoints.keys) {
            lanePoints[cs]!!.forEach { it.crossingSection = cs }
        }
        crossing.sections.addAll(lanePoints.keys)

        val northSouth = lanePoints[CrossingSection(crossing, 0)]!!
        val westEast = lanePoints[CrossingSection(crossing, 3)]!!

        val northVehicle = Vehicle()
        northVehicle.route.addAll(northSouth)
        northVehicle.position.set(0.0, 0.0, -60.0)
        northVehicle.rotation.rotationY((PI * 0.5).toFloat())
        northVehicle.velocity.set(0.0, 0.0, 8.0)
        northVehicle.maxVelocity = 8.0

        val westVehicle = Vehicle()
        westVehicle.route.addAll(westEast)
        westVehicle.position.set(-60.0, 0.0, 0.0)
        westVehicle.rotation.rotationY(0f)
        westVehicle.velocity.set(8.0, 0.0, 0.0)
        westVehicle.maxVelocity = 8.0

        northVehicle.nearby.add(westVehicle)
        westVehicle.nearby.add(northVehicle)

        val dt = 0.05f
        for (i in 0 until 250) {
            northVehicle.update(dt)
            westVehicle.update(dt)
        }

        assertFalse(
            northVehicle.position.z > 0.0 && westVehicle.position.x > 0.0,
            "Vehicles should not pass through the crossing and end up on the far side"
        )
    }

    private fun createFourWayLanes(crossing: Crossing): Map<CrossingSection, List<Lane>> {
        // North to South (section 0)
        val pNS0 = createLanePoint(0.0, -100.0, PI * 0.5)
        val pNS1 = createLanePoint(0.0, -15.0, PI * 0.5)
        val pNS2 = createLanePoint(0.0, 15.0, PI * 0.5)
        val pNS3 = createLanePoint(0.0, 100.0, PI * 0.5)
        val laneNS = createLane(pNS0, pNS1, pNS2)
        val laneNS2 = extendLane(pNS2, pNS3)

        // East to West (section 1)
        val pEW0 = createLanePoint(100.0, 0.0, PI)
        val pEW1 = createLanePoint(15.0, 0.0, PI)
        val pEW2 = createLanePoint(-15.0, 0.0, PI)
        val pEW3 = createLanePoint(-100.0, 0.0, PI)
        val laneEW = createLane(pEW0, pEW1, pEW2)
        val laneEW2 = extendLane(pEW2, pEW3)

        // South to North (section 2)
        val pSN0 = createLanePoint(0.0, 100.0, -PI * 0.5)
        val pSN1 = createLanePoint(0.0, 15.0, -PI * 0.5)
        val pSN2 = createLanePoint(0.0, -15.0, -PI * 0.5)
        val pSN3 = createLanePoint(0.0, -100.0, -PI * 0.5)
        val laneSN = createLane(pSN0, pSN1, pSN2)
        val laneSN2 = extendLane(pSN2, pSN3)

        // West to East (section 3)
        val pWE0 = createLanePoint(-100.0, 0.0, 0.0)
        val pWE1 = createLanePoint(-15.0, 0.0, 0.0)
        val pWE2 = createLanePoint(15.0, 0.0, 0.0)
        val pWE3 = createLanePoint(100.0, 0.0, 0.0)
        val laneWE = createLane(pWE0, pWE1, pWE2)
        val laneWE2 = extendLane(pWE2, pWE3)

        val cs0 = CrossingSection(crossing, 0)
        val cs1 = CrossingSection(crossing, 1)
        val cs2 = CrossingSection(crossing, 2)
        val cs3 = CrossingSection(crossing, 3)

        return mapOf(
            cs0 to listOf(laneNS, laneNS2),
            cs1 to listOf(laneEW, laneEW2),
            cs2 to listOf(laneSN, laneSN2),
            cs3 to listOf(laneWE, laneWE2)
        )
    }
}
