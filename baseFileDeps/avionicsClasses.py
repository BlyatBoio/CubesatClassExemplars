# Defines GPS object
class GPS:

    # GPS constructor
    def __init__(self, Latitude, Longitude, Altitude, Speed, Satellites, DOP):

        self.Latitude = Latitude
        self.Longitude = Longitude
        self.Altitude = Altitude
        self.Speed = Speed
        self.Satellites = Satellites
        self.DOP = DOP

    # Defines how GPS will be printed
    def __repr__(self):

        try:

            return "<Lat: {:.6f} degrees, Long: {:.6f} degrees, Altitude: {:.2f} m, Speed: {:.2f} m/s, Satellites: {}, DOP: {}>".format(
                self.Latitude,
                self.Longitude,
                self.Altitude,
                self.Speed,
                self.Satellites,
                self.DOP)

        except:

            return "<Lat: {} degrees, Long: {} degrees, Altitude: {} m, Speed: {} m/s, Satellites: {}, DOP: {}>".format(
                self.Latitude,
                self.Longitude,
                self.Altitude,
                self.Speed,
                self.Satellites,
                self.DOP)


# Defines ALTIMETER object
class ALTIMETER:

    # ALTIMETER constructor
    def __init__(self, Altitude, Temperature, Pressure, Humidity, Gas):

        self.Altitude = Altitude
        self.Temperature = Temperature
        self.Pressure = Pressure
        self.Humidity = Humidity
        self.Gas = Gas

    # Defines how ALTIMETER will be printed
    def __repr__(self):

        try:

            return "<Altitude: {:.2f} m, Temperature: {:.2f} C, Pressure: {:.2f} kPa, Humidity {:.2f} %, Gas: {:.0f} ohms>".format(
                self.Altitude,
                self.Temperature,
                self.Pressure,
                self.Humidity,
                self.Gas)

        except:

            return "<Altitude: {} m, Temperature: {} C, Pressure: {} kPa, Humidity {} %, Gas: {} ohms>".format(
                self.Altitude,
                self.Temperature,
                self.Pressure,
                self.Humidity,
                self.Gas)


# Defines ACCELEROMETER object
class ACCELEROMETER:

    # ACCELEROMETER constructor
    def __init__(self, X, Y, Z):

        self.X = X
        self.Y = Y
        self.Z = Z

    # Defines how ACCELEROMETER will be printed
    def __repr__(self):

        try:

            return "<X-Axis Acceleration: {:.2f} m/s^2, Y-Axis Acceleration: {:.2f} m/s^2, Z-Axis Acceleration: {:.2f} m/s^2>".format(
                self.X,
                self.Y,
                self.Z)

        except:

            return "<X-Axis Acceleration: {} m/s^2, Y-Axis Acceleration: {} m/s^2, Z-Axis Acceleration: {} m/s^2>".format(
                self.X,
                self.Y,
                self.Z)


# Defines GYROSCOPE object
class GYROSCOPE:

    # GYROSCOPE constructor
    def __init__(self, X, Y, Z):

        self.X = X
        self.Y = Y
        self.Z = Z

    # Defines how GYROSCOPE will be printed
    def __repr__(self):

        try:

            return "<X-Axis Rotational Velocity: {:.2f} radians/s, Y-Axis Rotational Velocity: {:.2f} radians/s, Z-Axis Rotational Velocity: {:.2f} radians/s>".format(
                self.X,
                self.Y,
                self.Z)

        except:

            return "<X-Axis Rotational Velocity: {} radians/s, Y-Axis Rotational Velocity: {} radians/s, Z-Axis Rotational Velocity: {} radians/s>".format(
                self.X,
                self.Y,
                self.Z)


# Defines MAGNETOMETER object
class MAGNETOMETER:

    # MAGNETOMETER constructor
    def __init__(self, X, Y, Z):

        self.X = X
        self.Y = Y
        self.Z = Z

    # Defines how MAGNETOMETER will be printed
    def __repr__(self):

        try:

            return "<X-Axis Field Strength: {:.2f} uT, Y-Axis Field Strength: {:.2f} uT, Z-Axis Field Strength: {:.2f} uT>".format(
                self.X,
                self.Y,
                self.Z)

        except:

            return "<X-Axis Field Strength: {} uT, Y-Axis Field Strength: {} uT, Z-Axis Field Strength: {} uT>".format(
                self.X,
                self.Y,
                self.Z)


# Defines POWER object
class POWER:

    # POWER constructor
    def __init__(self, voltage, current, wattage):

        self.Voltage = voltage
        self.Current = current
        self.Wattage = wattage

    # Defines how POWER will be printed
    def __repr__(self):

        try:

            return "<Voltage: {:.3} V, Current: {:.3} A, Wattage: {:.3} W>".format(self.Voltage,
                                                                                   self.Current,
                                                                                   self.Wattage)

        except:

            return "<Voltage: {} V, Current: {} A, Wattage: {} W>".format(self.Voltage,
                                                                          self.Current,
                                                                          self.Wattage)


# Defines BATTERY object
class BATTERY:

    # BATTERY constructor
    def __init__(self, voltage, percentage):

        self.Voltage = voltage
        self.Percentage = percentage

    # Defines how BATTERY will be printed
    def __repr__(self):

        try:

            return "<Voltage: {:.2f} V, Percentage: {:.2f} %>".format(self.Voltage,
                                                                      self.Percentage)

        except:

            return "<Voltage: {} V, Percentage: {} %>".format(self.Voltage,
                                                              self.Percentage)


# Defines SOLARPANEL object
class SOLARPANEL:

    # SOLARPANEL constructor
    def __init__(self, one, two, three):
        self.One = one
        self.Two = two
        self.Three = three

    # Defines how SOLARPANEL will be printed
    def __repr__(self):
        return "Solar Panel One: {},\nSolar Panel Two: {},\nSolar Panel Three:{}".format(self.One,
                                                                                         self.Two,
                                                                                         self.Three)


# Defines ANALOG object
class ANALOG:

    # ANALOG constructor
    def __init__(self, A0, A1, A2, A3, A4, A5, A6):

        self.A0 = A0
        self.A1 = A1
        self.A2 = A2
        self.A3 = A3
        self.A4 = A4
        self.A5 = A5
        self.A6 = A6

    # Defines how ANALOG will be printed
    def __repr__(self):

        try:

            return "<A0: {:.4f} V, A1: {:.4f} V, A2: {:.4f} V, A3: {:.4f} V, A4: {:.4f} V, A5: {:.4f} V, A6: {:.4f} V>".format(
                self.A0,
                self.A1,
                self.A2,
                self.A3,
                self.A4,
                self.A5,
                self.A6)

        except:

            return "<A0: {} V, A1: {} V, A2: {} V, A3: {} V, A4: {} V, A5: {} V, A6: {} V>".format(self.A0,
                                                                                                   self.A1,
                                                                                                   self.A2,
                                                                                                   self.A3,
                                                                                                   self.A4,
                                                                                                   self.A5,
                                                                                                   self.A6)


# Defines AVIONICSDATA object
class AVIONICSDATA:

    # AVIONICSDATA constructor
    def __init__(self, TIME, GPS, ALTIMETER, ACCELEROMETER, GYROSCOPE, MAGNETOMETER, POWERDRAW, SOLARPANEL, BATTERY,
                 ANALOG):
        self.TIME = TIME
        self.GPS = GPS
        self.ALTIMETER = ALTIMETER
        self.ACCELEROMETER = ACCELEROMETER
        self.GYROSCOPE = GYROSCOPE
        self.MAGNETOMETER = MAGNETOMETER
        self.POWERDRAW = POWERDRAW
        self.SOLARPANEL = SOLARPANEL
        self.BATTERY = BATTERY
        self.ANALOG = ANALOG

    # Defines how AVIONICSDATA will be printed
    def __repr__(self):
        return "{}\nGPS: {},\nAltimeter: {},\nAccelerometer: {},\nGyroscope: {},\nMagnetometer: {},\nSystem Power Draw: {},\n{},\nBattery: {},\nAnalog: {}".format(
            self.TIME,
            self.GPS,
            self.ALTIMETER,
            self.ACCELEROMETER,
            self.GYROSCOPE,
            self.MAGNETOMETER,
            self.POWERDRAW,
            self.SOLARPANEL,
            self.BATTERY,
            self.ANALOG)