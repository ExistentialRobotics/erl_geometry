import sympy as sp


def main():
    # second order curve
    # 3D: a * x^2 + b * x * y + c * y^2 + d * x * z + e * y * z + f * z^2 + g * x + h * y + i * z + j = 0
    a, b, c, d, e, f, g, h, i, j = sp.symbols("a b c d e f g h i j")
    px, py, pz, vx, vy, vz = sp.symbols("px py pz vx vy vz")
    t = sp.symbols("t")

    x = px + t * vx
    y = py + t * vy
    z = pz + t * vz
    curve = a * x**2 + b * x * y + c * y**2 + d * x * z + e * y * z + f * z**2 + g * x + h * y + i * z + j

    sol = sp.solve(curve, t)
    sp.init_printing(wrap_line=False)
    print(sp.simplify(sol[0]))
    print(sp.simplify(sol[1]))

    curve = g * x + h * y + i * z + j
    sol = sp.solve(curve, t)
    print(sp.simplify(sol[0]))

    # (
    #     -2 * a * px * vx
    #     - b * px * vy
    #     - b * py * vx
    #     - 2 * c * py * vy
    #     - d * px * vz
    #     - d * pz * vx
    #     - e * py * vz
    #     - e * pz * vy
    #     - 2 * f * pz * vz
    #     - g * vx
    #     - h * vy
    #     - i * vz
    #     - sp.sqrt(
    #         -4 * a * c * px**2 * vy**2
    #         + 8 * a * c * px * py * vx * vy
    #         - 4 * a * c * py**2 * vx**2
    #         - 4 * a * e * px**2 * vy * vz
    #         + 4 * a * e * px * py * vx * vz
    #         + 4 * a * e * px * pz * vx * vy
    #         - 4 * a * e * py * pz * vx**2
    #         - 4 * a * f * px**2 * vz**2
    #         + 8 * a * f * px * pz * vx * vz
    #         - 4 * a * f * pz**2 * vx**2
    #         + 4 * a * h * px * vx * vy
    #         - 4 * a * h * py * vx**2
    #         + 4 * a * i * px * vx * vz
    #         - 4 * a * i * pz * vx**2
    #         - 4 * a * j * vx**2
    #         + b**2 * px**2 * vy**2
    #         - 2 * b**2 * px * py * vx * vy
    #         + b**2 * py**2 * vx**2
    #         + 2 * b * d * px**2 * vy * vz
    #         - 2 * b * d * px * py * vx * vz
    #         - 2 * b * d * px * pz * vx * vy
    #         + 2 * b * d * py * pz * vx**2
    #         - 2 * b * e * px * py * vy * vz
    #         + 2 * b * e * px * pz * vy**2
    #         + 2 * b * e * py**2 * vx * vz
    #         - 2 * b * e * py * pz * vx * vy
    #         - 4 * b * f * px * py * vz**2
    #         + 4 * b * f * px * pz * vy * vz
    #         + 4 * b * f * py * pz * vx * vz
    #         - 4 * b * f * pz**2 * vx * vy
    #         - 2 * b * g * px * vx * vy
    #         + 2 * b * g * py * vx**2
    #         + 2 * b * h * px * vy**2
    #         - 2 * b * h * py * vx * vy
    #         + 2 * b * i * px * vy * vz
    #         + 2 * b * i * py * vx * vz
    #         - 4 * b * i * pz * vx * vy
    #         - 4 * b * j * vx * vy
    #         + 4 * c * d * px * py * vy * vz
    #         - 4 * c * d * px * pz * vy**2
    #         - 4 * c * d * py**2 * vx * vz
    #         + 4 * c * d * py * pz * vx * vy
    #         - 4 * c * f * py**2 * vz**2
    #         + 8 * c * f * py * pz * vy * vz
    #         - 4 * c * f * pz**2 * vy**2
    #         - 4 * c * g * px * vy**2
    #         + 4 * c * g * py * vx * vy
    #         + 4 * c * i * py * vy * vz
    #         - 4 * c * i * pz * vy**2
    #         - 4 * c * j * vy**2
    #         + d**2 * px**2 * vz**2
    #         - 2 * d**2 * px * pz * vx * vz
    #         + d**2 * pz**2 * vx**2
    #         + 2 * d * e * px * py * vz**2
    #         - 2 * d * e * px * pz * vy * vz
    #         - 2 * d * e * py * pz * vx * vz
    #         + 2 * d * e * pz**2 * vx * vy
    #         - 2 * d * g * px * vx * vz
    #         + 2 * d * g * pz * vx**2
    #         + 2 * d * h * px * vy * vz
    #         - 4 * d * h * py * vx * vz
    #         + 2 * d * h * pz * vx * vy
    #         + 2 * d * i * px * vz**2
    #         - 2 * d * i * pz * vx * vz
    #         - 4 * d * j * vx * vz
    #         + e**2 * py**2 * vz**2
    #         - 2 * e**2 * py * pz * vy * vz
    #         + e**2 * pz**2 * vy**2
    #         - 4 * e * g * px * vy * vz
    #         + 2 * e * g * py * vx * vz
    #         + 2 * e * g * pz * vx * vy
    #         - 2 * e * h * py * vy * vz
    #         + 2 * e * h * pz * vy**2
    #         + 2 * e * i * py * vz**2
    #         - 2 * e * i * pz * vy * vz
    #         - 4 * e * j * vy * vz
    #         - 4 * f * g * px * vz**2
    #         + 4 * f * g * pz * vx * vz
    #         - 4 * f * h * py * vz**2
    #         + 4 * f * h * pz * vy * vz
    #         - 4 * f * j * vz**2
    #         + g**2 * vx**2
    #         + 2 * g * h * vx * vy
    #         + 2 * g * i * vx * vz
    #         + h**2 * vy**2
    #         + 2 * h * i * vy * vz
    #         + i**2 * vz**2
    #     )
    # ) / (2 * (a * vx**2 + b * vx * vy + c * vy**2 + d * vx * vz + e * vy * vz + f * vz**2))

    # (
    #     -2 * a * px * vx
    #     - b * px * vy
    #     - b * py * vx
    #     - 2 * c * py * vy
    #     - d * px * vz
    #     - d * pz * vx
    #     - e * py * vz
    #     - e * pz * vy
    #     - 2 * f * pz * vz
    #     - g * vx
    #     - h * vy
    #     - i * vz
    #     + sp.sqrt(
    #         -4 * a * c * px**2 * vy**2
    #         + 8 * a * c * px * py * vx * vy
    #         - 4 * a * c * py**2 * vx**2
    #         - 4 * a * e * px**2 * vy * vz
    #         + 4 * a * e * px * py * vx * vz
    #         + 4 * a * e * px * pz * vx * vy
    #         - 4 * a * e * py * pz * vx**2
    #         - 4 * a * f * px**2 * vz**2
    #         + 8 * a * f * px * pz * vx * vz
    #         - 4 * a * f * pz**2 * vx**2
    #         + 4 * a * h * px * vx * vy
    #         - 4 * a * h * py * vx**2
    #         + 4 * a * i * px * vx * vz
    #         - 4 * a * i * pz * vx**2
    #         - 4 * a * j * vx**2
    #         + b**2 * px**2 * vy**2
    #         - 2 * b**2 * px * py * vx * vy
    #         + b**2 * py**2 * vx**2
    #         + 2 * b * d * px**2 * vy * vz
    #         - 2 * b * d * px * py * vx * vz
    #         - 2 * b * d * px * pz * vx * vy
    #         + 2 * b * d * py * pz * vx**2
    #         - 2 * b * e * px * py * vy * vz
    #         + 2 * b * e * px * pz * vy**2
    #         + 2 * b * e * py**2 * vx * vz
    #         - 2 * b * e * py * pz * vx * vy
    #         - 4 * b * f * px * py * vz**2
    #         + 4 * b * f * px * pz * vy * vz
    #         + 4 * b * f * py * pz * vx * vz
    #         - 4 * b * f * pz**2 * vx * vy
    #         - 2 * b * g * px * vx * vy
    #         + 2 * b * g * py * vx**2
    #         + 2 * b * h * px * vy**2
    #         - 2 * b * h * py * vx * vy
    #         + 2 * b * i * px * vy * vz
    #         + 2 * b * i * py * vx * vz
    #         - 4 * b * i * pz * vx * vy
    #         - 4 * b * j * vx * vy
    #         + 4 * c * d * px * py * vy * vz
    #         - 4 * c * d * px * pz * vy**2
    #         - 4 * c * d * py**2 * vx * vz
    #         + 4 * c * d * py * pz * vx * vy
    #         - 4 * c * f * py**2 * vz**2
    #         + 8 * c * f * py * pz * vy * vz
    #         - 4 * c * f * pz**2 * vy**2
    #         - 4 * c * g * px * vy**2
    #         + 4 * c * g * py * vx * vy
    #         + 4 * c * i * py * vy * vz
    #         - 4 * c * i * pz * vy**2
    #         - 4 * c * j * vy**2
    #         + d**2 * px**2 * vz**2
    #         - 2 * d**2 * px * pz * vx * vz
    #         + d**2 * pz**2 * vx**2
    #         + 2 * d * e * px * py * vz**2
    #         - 2 * d * e * px * pz * vy * vz
    #         - 2 * d * e * py * pz * vx * vz
    #         + 2 * d * e * pz**2 * vx * vy
    #         - 2 * d * g * px * vx * vz
    #         + 2 * d * g * pz * vx**2
    #         + 2 * d * h * px * vy * vz
    #         - 4 * d * h * py * vx * vz
    #         + 2 * d * h * pz * vx * vy
    #         + 2 * d * i * px * vz**2
    #         - 2 * d * i * pz * vx * vz
    #         - 4 * d * j * vx * vz
    #         + e**2 * py**2 * vz**2
    #         - 2 * e**2 * py * pz * vy * vz
    #         + e**2 * pz**2 * vy**2
    #         - 4 * e * g * px * vy * vz
    #         + 2 * e * g * py * vx * vz
    #         + 2 * e * g * pz * vx * vy
    #         - 2 * e * h * py * vy * vz
    #         + 2 * e * h * pz * vy**2
    #         + 2 * e * i * py * vz**2
    #         - 2 * e * i * pz * vy * vz
    #         - 4 * e * j * vy * vz
    #         - 4 * f * g * px * vz**2
    #         + 4 * f * g * pz * vx * vz
    #         - 4 * f * h * py * vz**2
    #         + 4 * f * h * pz * vy * vz
    #         - 4 * f * j * vz**2
    #         + g**2 * vx**2
    #         + 2 * g * h * vx * vy
    #         + 2 * g * i * vx * vz
    #         + h**2 * vy**2
    #         + 2 * h * i * vy * vz
    #         + i**2 * vz**2
    #     )
    # ) / (2 * (a * vx**2 + b * vx * vy + c * vy**2 + d * vx * vz + e * vy * vz + f * vz**2))

    # (-g * px - h * py - i * pz - j) / (g * vx + h * vy + i * vz)


if __name__ == "__main__":
    main()
