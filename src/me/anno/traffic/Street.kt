package me.anno.traffic

import me.anno.traffic.utils.SplineMaths.laneLength
import me.anno.traffic.utils.SplineMaths.lerp3
import org.joml.Vector3d

/**
 * a structure for quickly editing lanes
 * */
data class Street(
    val from: StreetPoint,
    val control: StreetPoint,
    val to: StreetPoint,
    // todo lanes may be added later on
    val lanes: ArrayList<Lane>
) {

    companion object {
        val defaultStreetDesign = IntArray(6)
    }

    val approxLength = laneLength(from.position, control.position, to.position)

    // encoded information about each lane
    var streetDesign: IntArray = defaultStreetDesign

    fun getPosition(t: Double, dst: Vector3d): Vector3d {
        return lerp3(
            from.position,
            control.position,
            to.position,
            t, dst
        )
    }
}
