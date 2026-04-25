package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.maths.Maths
import me.anno.maths.Maths.PIf
import me.anno.maths.Maths.mixAngle
import me.anno.maths.Maths.sq
import me.anno.maths.optimization.GoldenSectionSearch
import me.anno.traffic.*
import me.anno.traffic.vehicle.attachTrailer
import me.anno.traffic.vehicle.setOn
import me.anno.traffic.visuals.RandomNavigator.Companion.extendRoute
import me.anno.traffic.visuals.StreetMeshBuilder.addStreetMesh
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.atan2

fun createIntersection(
    network: Network, scene: Entity,
    streets: List<Street>,
    center: Vector3d, radius: Double,
    builder: StreetBuilder
) {

    fun isInside(lp: LanePoint): Boolean {
        return center.distanceSquared(lp.position) < sq(radius)
    }

    // define all lane-combinations...
    // find, which lanes need to be connected to others...
    //  and then create meshes for them
    val lanes = streets.flatMap { it.lanes }
    val laneToStreet = streets
        .flatMap { it.lanes.map { lane -> lane to it } }
        .toMap()
    val entryLanes = lanes.filter { !isInside(it.from) && isInside(it.to) }
    val exitLanes = lanes.filter { isInside(it.from) && !isInside(it.to) }

    val crossing = Crossing(center, radius)
    val crossingSections = HashMap<Street?, CrossingSection>()
    val center1 = crossingSections.getOrPut(null) { CrossingSection(crossing, 0) }

    // todo split crossing into sections, so each part can drive at a time...

    val newLanes = ArrayList<Lane>()
    for (entry in entryLanes) {
        for (exit in exitLanes) {
            val entryPoint = entry.to
            val exitPoint = exit.from

            if (absAngleDiff(entryPoint.angle - exitPoint.angle) > PIf - 0.1f) {
                continue
            }

            val entrySection = crossingSections.getOrPut(laneToStreet[entry]!!) {
                CrossingSection(crossing, crossingSections.size)
            }
            entry.crossingSection = entrySection

            val distance0 = entryPoint.position.distance(exitPoint.position)
            val entryDir = Vector3d(entry.getDirection1())
            val exitDir = Vector3d(exit.getDirection0()).negate()
            val distance = GoldenSectionSearch.goldenSectionSearch(0.3, 1.0, 0.01, { fac ->
                val distance = distance0 * fac
                entryDir.normalize(distance)
                exitDir.normalize(distance)
                val entryExtended = entryPoint.position + entryDir
                val exitExtended = exitPoint.position + exitDir
                entryExtended.distanceSquared(exitExtended)
            }) * distance0
            entryDir.normalize(distance)
            exitDir.normalize(distance)
            val entryExtended = entryPoint.position + entryDir
            val exitExtended = exitPoint.position + exitDir

            val centerPoint = Vector3d(entryExtended)
                .mix(exitExtended, 0.5)

            val angle = mixAngle(entryPoint.angle, exitPoint.angle, 0.5)
            val angleX = atan2(
                exitPoint.position.y - entryPoint.position.y,
                exitPoint.position.distance(entryPoint.position)
            ).toFloat()
            val controlRot = Quaternionf().rotateYXZ(angle.toFloat(), angleX, 0f)
            val control = LanePoint(centerPoint, controlRot, angle, builder.laneWidth)

            val lane = Lane(entryPoint, control, exitPoint)
            network.addLane(lane)
            newLanes.add(lane)

            lane.crossingSection = center1

        }
    }

    crossing.sections.addAll(crossingSections.values)

    addStreetMesh(newLanes, scene)
    debugDrawCircle(center, radius)

    network.addCrossing(crossing)
    for (lane in newLanes) {
        network.addLane(lane)
    }
}

fun spawnVehicles(network: Network, streets: List<Street>) {
    for (i in streets.indices) {
        val street = streets[i]
        for (lane in street.lanes) {
            val ts = 7
            for (ti in 0 until ts) {
                if (Maths.random() < 0.3f) continue

                val vehicle = Vehicle()
                val t = (ti + 0.5f) / ts
                vehicle.setOn(lane, t)

                for (i in 0 until 3) {
                    extendRoute(vehicle)
                }

                network.addVehicle(vehicle)

                var last = vehicle
                var chance = 0.1f
                while (Maths.random() < chance) {

                    vehicle.maxAcceleration *= 0.7f
                    vehicle.maxDeceleration *= 0.7f

                    val trailer = Vehicle()
                    last.attachTrailer(trailer, 2f, 2f)

                    network.addVehicle(trailer)
                    last = trailer
                    chance = 0.2f
                }
            }
        }
    }
}
