package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.maths.Maths
import me.anno.maths.Maths.PIf
import me.anno.maths.Maths.mixAngle
import me.anno.maths.Maths.sq
import me.anno.maths.optimization.GoldenSectionSearch
import me.anno.traffic.Crossing
import me.anno.traffic.CrossingSection
import me.anno.traffic.Lane
import me.anno.traffic.LanePoint
import me.anno.traffic.Network
import me.anno.traffic.Street
import me.anno.traffic.Vehicle
import me.anno.traffic.visuals.StreetMeshBuilder.addStreetMesh
import org.joml.Quaternionf
import org.joml.Vector3d


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
            val distance = GoldenSectionSearch.goldenSectionSearch(0.3, 1.0, 0.01, { fac ->
                val distance = distance0 * fac
                val entryDir = (entry.getPosition(1.01, 0.0, 0.0, Vector3d()) - entryPoint.position).normalize(distance)
                val exitDir = (exit.getPosition(-0.01, 0.0, 0.0, Vector3d()) - exitPoint.position).normalize(distance)
                val entryExtended = entryPoint.position + entryDir
                val exitExtended = exitPoint.position + exitDir
                entryExtended.distanceSquared(exitExtended)
            }) * distance0
            val entryDir = (entry.getPosition(1.01, 0.0, 0.0, Vector3d()) - entryPoint.position).normalize(distance)
            val exitDir = (exit.getPosition(-0.01, 0.0, 0.0, Vector3d()) - exitPoint.position).normalize(distance)
            val entryExtended = entryPoint.position + entryDir
            val exitExtended = exitPoint.position + exitDir

            val centerPoint = Vector3d(entryExtended)
                .mix(exitExtended, 0.5)

            val angle = mixAngle(entryPoint.angle, exitPoint.angle, 0.5)
            val control = LanePoint(centerPoint, Quaternionf().rotateY(angle.toFloat()), angle, builder.laneWidth)

            val lane = Lane(entryPoint, control, exitPoint)
            network.addLane(lane)
            newLanes.add(lane)

            lane.crossingSection = center1

        }
    }

    crossing.sections.addAll(crossingSections.values)

    addStreetMesh(Street(newLanes), scene)
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
                val t = (ti + 0.5) / ts
                lane.getPosition(t, 0.0, 0.0, vehicle.position)
                lane.getRotation(t.toFloat(), vehicle.rotation).rotateY(PIf) // why is this 180° necessary??
                vehicle.updateDirections()
                vehicle.route.add(lane)

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
                    trailer.position.set(last.position)
                    trailer.rotation.set(last.rotation)
                    trailer.updateDirections()

                    last.attachTrailer(trailer, 2f, 2f)
                    network.addVehicle(trailer)
                    last = trailer
                    chance = 0.2f
                }

            }
        }
    }
}

fun extendRoute(vehicle: Vehicle) {
    var curr = vehicle.route.last()
    curr = curr.to.lanes
        .filter { it.from == curr.to }
        .randomOrNull() ?: return
    vehicle.route.add(curr)
}
