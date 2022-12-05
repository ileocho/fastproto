from djitellopy import Tello

tello = Tello()

tello.connect()
tello.takeoff()
tello.move_up(20)
tello.rotate_counter_clockwise(360)
tello.land() 