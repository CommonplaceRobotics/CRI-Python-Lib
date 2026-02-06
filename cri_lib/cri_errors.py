class CRIError(Exception):
    """Base class for CRI-related errors."""


class CRIConnectionError(CRIError):
    """Raised when there is a connection error."""

    def __init__(self, message="Not connected to iRC or connection lost."):
        self.message = message
        super().__init__(self.message)


class CRICommandError(CRIError):
    """Raised when a command fails to execute properly."""

    def __init__(self, message="Command execution failed."):
        self.message = message
        super().__init__(self.message)


class CRICommandTimeOutError(CRIError):
    """Raised when a command times out."""

    def __init__(self, message="Time out waiting for command response."):
        self.message = message
        super().__init__(self.message)
