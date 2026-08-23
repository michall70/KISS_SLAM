import numpy as np

def exp_map(xi):
    '''
    xi must be a row vector with 6 elements
    xi = [rho, phi]
    '''

    rho = xi[0:3]
    phi = xi[3:6]
    theta = np.linalg.norm(phi)
    t = rho     # approximately
    mat_phi = np.array(
        [[0, -phi[2], phi[1]],
         [phi[2], 0, -phi[0]],
         [-phi[1], phi[0], 0]]
    )

    if theta <= 1e-6:
        R = np.eye(3) + mat_phi
    else:
        mat_phi_sq = mat_phi @ mat_phi
        R = np.eye(3) + (np.sin(theta) / theta) * mat_phi + ((1 - np.cos(theta)) / np.square(theta)) * mat_phi_sq

    # print(R @ R.transpose())
    T = np.eye(4)
    T[:3, :3] = R       # 左上 3×3 放旋转
    T[:3, 3]  = t       # 右上放平移
    return T

if __name__ == "__main__":
    xi = [0,0,0, 0,0,np.pi/2]
    T = exp_map(xi)
    print(T)