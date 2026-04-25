package me.anno.traffic.vehicle

import me.anno.traffic.Vehicle

class VehicleLink(
    val engine: Vehicle,
    val linkToEngine: Float,
    val linkToTrailer: Float,
)