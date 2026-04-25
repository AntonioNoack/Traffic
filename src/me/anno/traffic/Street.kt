package me.anno.traffic

/**
 * a structure for quickly editing lanes
 * */
data class Street(
    val from: StreetPoint,
    val control: StreetPoint?,
    val to: StreetPoint,
    val lanes: ArrayList<Lane>
)