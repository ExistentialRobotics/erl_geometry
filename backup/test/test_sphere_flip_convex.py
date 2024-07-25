import numpy as np
import matplotlib.pyplot as plt


def get_circle_points(radius, n_points):
    theta = np.linspace(0, 2 * np.pi, n_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return np.stack([x, y], axis=1)


def sphere_flip(points, radius):
    norms = np.linalg.norm(points, axis=1, keepdims=True)
    projected = 2 * radius * points / norms - points
    return projected, get_circle_points(radius, 200)


def main():
    triangle = np.array([[1, 1], [1, -1], [0.5, 0]])
    triangle = np.concatenate(
        [
            np.linspace(triangle[0], triangle[1], 50),
            np.linspace(triangle[1], triangle[2], 50),
            np.linspace(triangle[2], triangle[0], 50),
        ]
    )

    r1 = 2
    r2 = 3.5

    projected1, circle1 = sphere_flip(triangle, r1)
    projected2, circle2 = sphere_flip(triangle, r2)

    plt.figure(figsize=(10, 10))
    plt.scatter(0, 0, label="view position")
    plt.scatter(triangle[:, 0], triangle[:, 1], label="object")
    plt.scatter(projected1[:, 0], projected1[:, 1], label="projected 1")
    plt.scatter(projected2[:, 0], projected2[:, 1], label="projected 2")
    plt.scatter(circle1[:, 0], circle1[:, 1], label="circle 1")
    plt.scatter(circle2[:, 0], circle2[:, 1], label="circle 2")
    plt.axis("equal")
    plt.legend()
    plt.show()

    pass


if __name__ == "__main__":
    main()
