package me.anno.traffic

import me.anno.traffic.utils.SplineMaths.laneLength

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
}
