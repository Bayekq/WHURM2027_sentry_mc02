import numpy as np
import pandas as pd
from scipy.optimize import curve_fit


def func_optimize_A6():
    
    # 定义要拟合的非线性函数
    def func_target(data, axx, ayy, azz, axy, axz, ayz, b0, b1, b2):
        vector_raw = np.array(data)
        matrix_A = np.array([[axx, axy, axz], [axy, ayy, ayz], [axz, ayz, azz]])
        vector_b = np.array([[b0], [b1], [b2]])

        vector_calib = matrix_A.dot(vector_raw) + vector_b
        
        norm_calib = np.linalg.norm(vector_calib, axis=0)

        return (norm_calib - 1)**2
    
    # 读取数据
    array_data = pd.read_csv('accel_calib.csv').values
    # 提取数据
    array_ax = array_data[:, 0] / 9.7947
    array_ay = array_data[:, 1] / 9.7947
    array_az = array_data[:, 2] / 9.7947
    # 测试一组输出用例
    test_data = np.array([array_ax[0], array_ay[0], array_az[0]]).reshape(3, 1)
    print(f"Test data: {test_data}")
    print(f"Function output for test data: {func_target(test_data, 1, 1, 1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)}\n\n")
    # 使用curve_fit进行参数优化
    params, covariance = curve_fit(
        func_target,
        np.array([array_ax, array_ay, array_az]),
        np.zeros(array_ax.shape),
        p0=[1, 1, 1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        maxfev=100000
    )
    # 输出优化结果
    print(f"Optimized parameters: {params}")
    print(f"Covariance of the parameters: {covariance}")
    print(f"axx: {params[0]}\nayy: {params[1]}\nazz: {params[2]}\naxy: {params[3]}\naxz: {params[4]}\nayz: {params[5]}\nb0: {params[6]}\nb1: {params[7]}\nb2: {params[8]}")


def func_optimize_A9():
    
    # 定义要拟合的非线性函数
    def func_target(data, axx, ayy, azz, axy, axz, ayz, ayx, azx, azy, b0, b1, b2):
        vector_raw = np.array(data)
        matrix_A = np.array([[axx, axy, axz], [ayx, ayy, ayz], [azx, azy, azz]])
        vector_b = np.array([[b0], [b1], [b2]])

        vector_calib = matrix_A.dot(vector_raw) + vector_b
        
        norm_calib = np.linalg.norm(vector_calib, axis=0)

        return (norm_calib - 1)**2
    
    # 读取数据
    array_data = pd.read_csv('accel_calib.csv').values
    # 提取数据
    array_ax = array_data[:, 0] / 9.7947
    array_ay = array_data[:, 1] / 9.7947
    array_az = array_data[:, 2] / 9.7947
    # 测试一组输出用例
    test_data = np.array([array_ax[0], array_ay[0], array_az[0]]).reshape(3, 1)
    print(f"Test data: {test_data}")
    print(f"Function output for test data: {func_target(test_data, 1, 1, 1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)}\n\n")
    # 使用curve_fit进行参数优化
    params, covariance = curve_fit(
        func_target,
        np.array([array_ax, array_ay, array_az]),
        np.zeros(array_ax.shape),
        p0=[1, 1, 1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        maxfev=100000
    )
    # 输出优化结果
    print(f"Optimized parameters: {params}")
    print(f"Covariance of the parameters: {covariance}")
    print(f"axx: {params[0]}\nayy: {params[1]}\nazz: {params[2]}\naxy: {params[3]}\naxz: {params[4]}\nayz: {params[5]}\nayx: {params[6]}\nazx: {params[7]}\nazy: {params[8]}\nb0: {params[9]}\nb1: {params[10]}\nb2: {params[11]}")

if __name__ == "__main__":
    func_optimize_A6()
    func_optimize_A9()