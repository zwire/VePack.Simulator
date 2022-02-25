import time, threading, airsim, tcp_socket

class VirtualVehicle:

    def __init__(self):

        self.car = airsim.CarClient()
        self.car.confirmConnection()
        self.car.enableApiControl(True, 'Car')
        self.operation = self.car.getCarControls('Car')
        self.operation.is_manual_gear = True
        self.throttle = 0
        self.isOpened = True

    def operate(self, rcv):

        if len(rcv) < 3: return
        throttle = float(rcv[0])
        steering = float(rcv[1])
        brake = int(rcv[2])
        self.operation.manual_gear = -1 if throttle < 0 else 1
        self.operation.throttle = abs(throttle)
        self.operation.steering = steering
        self.operation.brake = brake
        self.car.setCarControls(self.operation, 'Car')

    def getState(self):

        speed = self.car.getCarState('Car').speed
        imu = self.car.getImuData('Imu', 'Car')
        posture = airsim.utils.to_eularian_angles(imu.orientation)
        gps = self.car.getGpsData('Gps', 'Car').gnss.geo_point
        return [speed, gps.latitude, gps.longitude, posture[0], posture[1], posture[2]]

    def setMap(self, rcv):

        points = []
        for i in range((int)(len(rcv) / 2)):
            points.append(airsim.Vector3r(float(rcv[i * 2 + 0]), float(rcv[i * 2 + 1]), 1))
        self.car.simPlotPoints(points, color_rgba=[1, 0, 0, 0], size=5, duration=-1, is_persistent=True)


if __name__ == '__main__':

    vehicle = VirtualVehicle()
    sock = tcp_socket.TcpServer('127.0.0.1', 3000)
    stream = sock.get_stream()
    vehicle.setMap(stream.read().strip().split(','))
    lock = threading.Lock()

    def receivingLoop():
        while vehicle.isOpened:
            with lock:
                vehicle.operate(stream.read().strip().split(','))
                time.sleep(0.01)

    def sendingLoop():
        while vehicle.isOpened:
            with lock:
                state = vehicle.getState()
                stream.write(','.join(str(s) for s in state))
                time.sleep(0.01)

    th1 = threading.Thread(target=receivingLoop)
    th2 = threading.Thread(target=sendingLoop)
    th1.start()
    th2.start()
    th1.join()
    th2.join()

    stream.close()
    sock.close()