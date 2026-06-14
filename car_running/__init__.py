# 导入car_control模块中的类和函数
from .running import run

# 可以选择性地创建全局实例（不推荐，因为会自动执行）
# car = run()

# 定义包的公开接口
__all__ = ['run']

# 可选：提供一个工厂函数来创建实例
def create_car():
    """创建一个新的小车控制实例"""
    return run()