from email import message


class VehicleNotArmedException(Exception):
    """Exception raised when a vehicle is not armed but an operation requires it to be armed."""
    
    def __init__(self, name: str) -> None:
        message = f'Vehicle {name} is not armed. Please arm the vehicle before proceeding.'
        super().__init__(message)
        self.message = message