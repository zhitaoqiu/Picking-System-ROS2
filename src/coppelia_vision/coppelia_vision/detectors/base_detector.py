from abc import ABC, abstractmethod

class BaseDetector(ABC):
    """所有视觉模型的抽象基类"""
    
    def __init__(self, config: dict):
        self.config = config

    @abstractmethod
    def load_model(self):
        """加载模型权重"""
        pass

    @abstractmethod
    def infer(self, image):
        """
        推理接口
        :param image: OpenCV 格式图像 (BGR)
        :return: result (自定义的标准格式，例如字典列表)
        """
        pass