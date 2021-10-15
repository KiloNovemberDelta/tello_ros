import time


class PID:
    """ Simple PID control.
    """

    def __init__(self, Kp=1, Kd=0, Ki=0):
        # initialze gains
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.init()

    def setKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def setKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def setKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def setPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def init(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def loop(self, error):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        self.currtm = time.time()  # get t
        dt = self.currtm - self.prevtm  # get delta t
        de = error - self.prev_err  # get delta error

        self.Cp = self.Kp * error  # proportional term
        self.Ci += error * dt  # integral term

        self.Cd = 0
        if dt > 0:  # no div by zero
            self.Cd = de / dt  # derivative term

        self.prevtm = self.currtm  # save t for next pass
        self.prev_err = error  # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)
