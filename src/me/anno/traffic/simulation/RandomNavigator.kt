package me.anno.traffic.simulation

import me.anno.ecs.Component
import me.anno.ecs.systems.OnUpdate
import me.anno.traffic.Network
import me.anno.traffic.Vehicle

/**
 * the easiest navigation strategy: vehicles drive wherever they like
 * */
class RandomNavigator(val network: Network) : Component(), OnUpdate {

    companion object {
        fun extendRoute(vehicle: Vehicle) {
            var curr = vehicle.route.last()
            curr = curr.to.lanes
                .filter { it.from == curr.to }
                .randomOrNull() ?: return
            vehicle.addedLaneToRoute = true
            vehicle.route.add(curr)
        }
    }

    override fun onUpdate() {
        val vehicles = network.vehicles
        for (vehicle in vehicles) {
            vehicle.addedLaneToRoute = false
        }

        for (vehicle in vehicles) {
            if (vehicle.routeIndex > 0) {
                vehicle.route.removeFirst()
                vehicle.routeIndex--
            }

            val link = vehicle.linkToEngine
            if (link != null) {
                // trailers copy the behavior of their engine
                if (link.engine.addedLaneToRoute) {
                    vehicle.route.add(link.engine.route.last())
                    vehicle.addedLaneToRoute = true
                }
            } else if (vehicle.routeIndex + 3 > vehicle.route.size) {
                extendRoute(vehicle)
            }
        }
    }
}