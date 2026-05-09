package me.anno.traffic

/**
 * a small street, which is only used to get in/out of buildings...
 *   is only the start and end of navigation
 *
 * todo navigation: streets -> lanes + streetlets
 * todo test that driveways work correctly w.r.t. streetT
 * */
class Driveway(
    val streetT: Float, // exit/entry T of streetLane
    val streetLane: Lane,
    from: LanePoint,
    control: LanePoint,
    to: LanePoint
) : Lane(from, control, to)