package me.anno.traffic.testing

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.Mesh
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.maths.Maths.PIf
import me.anno.maths.Maths.clamp
import me.anno.maths.Maths.posMod
import me.anno.traffic.Network
import me.anno.traffic.Street
import me.anno.traffic.StreetPoint
import me.anno.traffic.editor.StreetBuilder
import me.anno.traffic.visuals.StreetMeshBuilder
import org.joml.Vector3d
import java.lang.Math.TAU
import kotlin.math.max
import kotlin.math.min

// todo this looks pretty wrong :(

fun buildFivePointIntersectionInPlannerMode(network: Network, scene: Entity): List<Street> {

    val raise = 0.0

    val n = 5
    val builder = StreetBuilder(network)

    val center = network.getOrPutPoint(Vector3d(0.0, raise + 1.0, 0.0))

    val outerPos0 = Vector3d(130.0, 1.0, -130.0)
    val outer = ArrayList<StreetPoint>()

    val streets = ArrayList<Street>()
    for (i in 0 until n) {
        val angle = i * TAU / n

        builder.position0.set(center.position)
        builder.position1.set(110.0, raise * 0.5 + 1.0, 0.0).rotateY(angle)
        builder.position2.set(outerPos0).rotateY(angle)

        val street = builder.placeStreetInPlannerMode()
        streets.add(street)

        check(street.from === center)
        outer.add(street.to)
    }

    for (i in 0 until n) {

        val angle1 = (i + 0.5) * TAU / n

        builder.position0.set(outer[i].position)
        builder.position1.set(outerPos0).rotateY(angle1)
        builder.position2.set(outer[posMod(i + 1, n)].position)

        builder.extrudeCenter(1.0)

        streets.add(builder.placeStreetInPlannerMode())
    }

    createMeshesByNetwork(network, scene)

    return streets
}

fun createMeshesByNetwork(network: Network, scene: Entity) {
    for (street in network.streets) {
        recalculateLanes(street, network)
    }

    network.streetPointTree.forEach { streetPoint ->
        createCrossingMesh(streetPoint, network, scene)
    }

    for (street in network.streets) {
        val mesh = createStreetMesh(street)
        scene.add(MeshComponent(mesh).apply { name = "Street" })
    }
}

// todo crossings in the future may have different types,
//  and may become really complicated, e.g. a crossing type may be cloverleaf,
//  or light-controlled intersection with Zebrastreifen

fun createCrossingMesh(streetPoint: StreetPoint, network: Network, scene: Entity) {
    val builder = StreetBuilder(network)
    val center = streetPoint.position
    val radius = getIntersectionRadius(streetPoint) * 1.5
    createIntersection(network, scene, streetPoint.streets, streetPoint, center, radius, builder)
}

fun createStreetMesh(street: Street): Mesh {
    return StreetMeshBuilder.createStreetMesh(street.lanes)
}

fun recalculateLanes(street: Street, network: Network) {
    // for each side, calculate crossing radius
    val insetFrom = getIntersectionRadius(street.from)
    val insetTo = getIntersectionRadius(street.to)
    // move back by that amount
    val insetFromT = min(insetFrom / street.approxLength, 0.45f)
    val insetToT = max(1f - insetTo / street.approxLength, 0.55f)
    // then create lanes there as usual
    val builder = StreetBuilder(network)
    builder.position0.set(street.from.position)
    builder.position1.set(street.control.position)
    builder.position2.set(street.to.position)
    builder.createStreetInExpertMode(insetFromT, insetToT, street)
    for (lane in street.lanes) {
        network.addLane(lane)
    }
}

fun getIntersectionRadius(streetPoint: StreetPoint): Float {
    if (streetPoint.streets.isEmpty()) return 1f

    val maxNumLanes = streetPoint.streets.maxOf { it.streetDesign.size }
    // intersection must be made larger/shifted, if two angles are close
    val allAngles = streetPoint.streets.flatMap { it.findAngles(streetPoint) }
    var minAngleDiff = PIf
    for (i in 1 until allAngles.size) {
        for (j in 0 until i) {
            val ai = allAngles[i]
            val aj = allAngles[j]
            val diff = absAngleDiff((ai - aj).toDouble()).toFloat()
            minAngleDiff = min(minAngleDiff, diff)
        }
    }
    return maxNumLanes * 4f * (2f - clamp(minAngleDiff))
}

fun Street.findAngles(streetPoint: StreetPoint): List<Float> {
    val result = ArrayList<Float>(2)
    if (from == streetPoint) result.add(from.position.angleYTo(control.position).toFloat())
    if (to == streetPoint) result.add(to.position.angleYTo(control.position).toFloat())
    return result
}