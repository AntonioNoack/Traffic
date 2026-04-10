package me.anno.traffic.editor

import me.anno.engine.raycast.RayQuery

interface BuildControls {
    fun raycast(ray: RayQuery)
    fun canPlaceStreet()
}