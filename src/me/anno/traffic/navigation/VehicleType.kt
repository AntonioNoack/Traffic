package me.anno.traffic.navigation

// todo should this be an enum?
enum class VehicleType(val minLaneWidth: Float) {

    PEDESTRIAN(0.8f),
    BIKE(1.2f),
    SCOOTER(1.0f),

    MOTORBIKE(1.5f),
    CAR(2.5f),
    TAXI(2.5f),
    BUS(3.2f),
    VAN(2.8f),
    TRUCK(3.2f),

    AMBULANCE(3.0f),
    FIRE_TRUCK(3.5f),
    POLICE_CAR(2.5f),

    CARGO_TRAIN(4.0f),
    PASSENGER_TRAIN(4.0f),
    TRAM(3.5f),
    SUBWAY(4.0f),
    MONORAIL(4.0f),
    CABLE_CAR(3.0f),

    CARGO_PLANE(50.0f),
    PASSENGER_PLANE(50.0f),

    // could be fire, police, news, tourists
    HELICOPTER(15.0f),

    CARGO_SHIP(30.0f),
    PASSENGER_SHIP(30.0f),
    FERRY(20.0f),
}