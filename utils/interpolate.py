import numpy as np
from scipy.interpolate import interp1d, interp2d


class Interp1:
    def __init__(self, x, y, kind="linear", bounds_error=False, fill_value='extrapolate'):
        """

        :param x: 插值的节点，横坐标
        :param y: 插值的节点，纵坐标
        :param kind: 插值模式{'linear', 'cubic', 'quintic'}
        :param bounds_error: 是否允许超过x边界的自变量，设为False则可以外推
        :param fill_value: 设置外推方式，如果不加此参数，则外推返回nan，'extrapolate'代表外推
        """
        self.interp_fun = interp1d(x, y, kind=kind, bounds_error=bounds_error, fill_value=fill_value)

    def pre(self, x):
        # x为数时返回数，x为列表时返回列表
        return self.interp_fun(x)


class Interp2:
    def __init__(self, x, y, value, kind='linear'):
        """
        输入：array
            data: 坐标[[x1, y1], [x2, y2], ...]
            value: 对应坐标矩阵的值[z1, z2, ...]
            kind: {'linear', 'cubic', 'quintic'}
        """
        self.interp_fun = interp2d(x, y, value, kind=kind)

    @classmethod
    def matrix_node(cls, data, nodex, nodey, kind='linear'):
        """
        对于规则的网格插值
        :param kind: 插值类型：线性、样条
        :param data: 插值的矩阵[m*n]
        :param nodex: 插值节点：一维长度为n的数组
        :param nodey: 插值节点：一维长度为m的数组
        :return: 插值对象
        """
        points = np.array([[s, k] for k in nodey for s in nodex])
        point_value = data.flatten(order='C')
        return cls(points[:, 0], points[:, 1], point_value, kind=kind)

    def pre(self, x, y):
        """
        预测值
        :param x: numpy.ndarray | number
        :param y: numpy.ndarray | number
        :return: 对应的预测数组或者是值
        """
        if isinstance(x, np.ndarray) | isinstance(y, np.ndarray):
            return self.pre_multi(x, y)
        else:
            return self.pre_one(x, y)

    def pre_one(self, x, y):
        return self.interp_fun(x, y)[0]

    def pre_multi(self, x, y):
        return self.interp_fun(x, y)


if __name__ == '__main__':
    # test interp1
    # test_a = np.array([1,2,3])
    # test_b = np.array([4,8,9])
    # interp = Interp1(test_a, test_b).interp_fun
    # print(interp([1.5]))

    # test interp2
    test_data = np.array([[1, 2, 3], [5, 9, 10]])
    test_x = np.array([3, 6, 7])
    test_y = np.array([1, 6])
    test_interp = Interp2.matrix_node(test_data, test_x, test_y)
    print(test_interp.pre_one(1.5, 2.5))
    print(test_interp.pre_multi(np.array([1, 2]), np.array([3, 4])))
