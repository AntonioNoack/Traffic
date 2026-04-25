package me.anno.traffic.vehicle

import me.anno.maths.Maths.absClamp
import me.anno.traffic.Vehicle

fun List<Vehicle>.update(dt: Float) {
    if (!(dt > 0f)) return

    for (i in indices) this[i].update0(dt)
    for (i in indices) this[i].update1(dt)

    // todo if(size>32) build a tree to resolve the collisions faster (~10x speedup?)
    for (i in indices) this[i].update2()
}

fun Vehicle.updateS(dt: Float) {
    if (!(dt > 0f)) return

    update0(dt)
    update1(dt)
    update2()
}

fun Vehicle.update0(dt: Float) {
    if (isCrashed) {
        applyDrivingAfterCrash(dt)
    } else if (linkToEngine == null) {
        applyMindfulDriving(dt)
    }
}

fun Vehicle.update1(dt: Float) {
    val link = linkToEngine
    if (link != null && !isCrashed) {
        applyTrailerFollowing(link, dt)
    }

    applyVelocity(dt)

    // Refresh bounds before collision tests so the SAT uses the current frame's pose,
    // not the previous frame's cached AABB.
    updateStrictBounds()
}

fun Vehicle.update2() {
    // Resolve collisions after motion, so frame-to-frame overlap is caught immediately
    // instead of only after one vehicle has already passed through the other.
    resolveCollisions()
}


fun Vehicle.applyVelocity(dt: Float) {
    angularVelocity = absClamp(angularVelocity, 100f)
    rotation.rotateY(angularVelocity * dt)
    rotation.normalize()
    check(rotation.isFinite) { "Invalid rotation by $angularVelocity * $dt" }

    updateDirections()

    position.fma(dt.toDouble(), velocity)
}
