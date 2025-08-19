class VehicleArmFailedException(Exception):
    """Exception raised when a vehicle fails to arm."""
    
    def __init__(self, name: str) -> None:
        message = f'Vehicle {name} failed to arm. Please check the vehicle status.'
        super().__init__(message)
        self.message = message

    def __str__(self):
        return f"VehicleArmFailedException: {self.message}"