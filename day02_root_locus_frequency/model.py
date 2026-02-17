import control


def get_transfer_function():
    """
    Plant:
        G(s) = 1 / (s (s + 2))
    """
    num = [1]
    den = [1, 2, 0]  # s^2 + 2s + 0
    return control.TransferFunction(num, den)