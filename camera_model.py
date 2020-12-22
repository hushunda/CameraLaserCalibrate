'''
相机模型
'''
class BaseCameraModel():
    def __init__(self):
        pass
    def load_camera_params(self):
        pass

class Omnidir():
    '''
    FOV >150
    '''
    def __init__(self):
        super(Omnidir()).__init__()
        pass

class Fish():
    '''
    120<FOV<150
    '''
    def __init__(self):
        super(Fish()).__init__()
        pass

class PinHole():
    '''
    FOV <120
    '''
    def __init__(self):
        super(PinHole()).__init__()
        pass

class CustomModel():
    def __init__(self):
        super(CustomModel()).__init__()
        pass