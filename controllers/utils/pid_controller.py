
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        """Initializes the PID controller with the specified gains and setpoint.
        Parameters:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            setpoint (float): The desired target value for the controller.
        Initializes internal variables for previous command, previous error, and integral term."""

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        # Variables pour le PID
        self.prev_command = 0
        self.prev_err = 0
        self.integral = 0

    def update(self, measurement) -> float:
        """
        Updates the PID controller with a new measurement and computes the control command.
        Args:
            measurement (float): The current measured value of the process variable.
        Returns:
            float: The computed control command based on the PID algorithm.
        Notes:
            - The method calculates the proportional, integral, and derivative terms using the current error,
              updates the integral and previous error states, and returns the sum as the control output.
            - Assumes that `self.setpoint`, `self.Kp`, `self.Ki`, `self.Kd`, `self.integral`, and `self.prev_err`
              are properly initialized attributes of the class.
        """
        meas = measurement
        err = meas - self.setpoint

        # Partie proportionnelle
        prop = self.Kp * err
        
        # Partie intégrale
        self.integral += self.Ki * err
        
        # Partie dérivée
        deriv = self.Kd*(err - self.prev_err)

        # Mise à jour de l'erreur précédente
        self.prev_err = err
        
        # Commande totale
        command = prop + self.integral + deriv
        
        return command