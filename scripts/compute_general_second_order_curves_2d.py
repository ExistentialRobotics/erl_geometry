import sympy as sp


def main():
    # second order curve
    # 2D: a * x^2 + b * x * y + c * y^2 + d * x + e * y + f = 0
    a, b, c, d, e, f = sp.symbols("a b c d e f")
    px, py, vx, vy = sp.symbols("px py vx vy")
    t = sp.symbols("t")

    x = px + t * vx
    y = py + t * vy
    curve = a * x**2 + b * x * y + c * y**2 + d * x + e * y + f

    sol = sp.solve(curve, t)
    sp.init_printing(wrap_line=False)
    print(sp.simplify(sol[0]))
    print(sp.simplify(sol[1]))

    sddf = sol[0]

    d_sddf_dx = sp.diff(sddf, px)
    d_sddf_dy = sp.diff(sddf, py)
    res = d_sddf_dx * vx + d_sddf_dy * vy
    res = sp.simplify(res)
    print("grad_p.T @ v:", res)

    d_sddf_dvx = sp.diff(sddf, vx)
    d_sddf_dvy = sp.diff(sddf, vy)
    res = d_sddf_dvx * vx + d_sddf_dvy * vy
    res = sp.simplify(sp.simplify(res))
    print("grad_v.T @ v:", res)

    for i in range(2):
        poly = sp.poly(sol[i])
        for mono in poly.monoms():
            term = sp.prod(x**k for x, k in zip(poly.gens, mono))
            sp.pretty_print(term)

    curve = d * x + e * y + f
    sol = sp.solve(curve, t)
    print(sp.simplify(sol[0]))

    sddf = sol[0]

    d_sddf_dx = sp.diff(sddf, px)
    d_sddf_dy = sp.diff(sddf, py)
    res = d_sddf_dx * vx + d_sddf_dy * vy
    res = sp.simplify(res)
    print("grad_p.T @ v:", res)

    d_sddf_dvx = sp.diff(sddf, vx)
    d_sddf_dvy = sp.diff(sddf, vy)
    res = d_sddf_dvx * vx + d_sddf_dvy * vy
    res = sp.simplify(res)
    print("grad_v.T @ v:", res)

    # (
    #     -2 * a * px * vx
    #     - b * px * vy
    #     - b * py * vx
    #     - 2 * c * py * vy
    #     - d * vx
    #     - e * vy
    #     - sp.sqrt(
    #         -4 * a * c * px**2 * vy**2
    #         + 8 * a * c * px * py * vx * vy
    #         - 4 * a * c * py**2 * vx**2
    #         + 4 * a * e * px * vx * vy
    #         - 4 * a * e * py * vx**2
    #         - 4 * a * f * vx**2
    #         + b**2 * px**2 * vy**2
    #         - 2 * b**2 * px * py * vx * vy
    #         + b**2 * py**2 * vx**2
    #         - 2 * b * d * px * vx * vy
    #         + 2 * b * d * py * vx**2
    #         + 2 * b * e * px * vy**2
    #         - 2 * b * e * py * vx * vy
    #         - 4 * b * f * vx * vy
    #         - 4 * c * d * px * vy**2
    #         + 4 * c * d * py * vx * vy
    #         - 4 * c * f * vy**2
    #         + d**2 * vx**2
    #         + 2 * d * e * vx * vy
    #         + e**2 * vy**2
    #     )
    # ) / (2 * (a * vx**2 + b * vx * vy + c * vy**2))

    # (
    #     -2 * a * px * vx
    #     - b * px * vy
    #     - b * py * vx
    #     - 2 * c * py * vy
    #     - d * vx
    #     - e * vy
    #     + sp.sqrt(
    #         -4 * a * c * px**2 * vy**2
    #         + 8 * a * c * px * py * vx * vy
    #         - 4 * a * c * py**2 * vx**2
    #         + 4 * a * e * px * vx * vy
    #         - 4 * a * e * py * vx**2
    #         - 4 * a * f * vx**2
    #         + b**2 * px**2 * vy**2
    #         - 2 * b**2 * px * py * vx * vy
    #         + b**2 * py**2 * vx**2
    #         - 2 * b * d * px * vx * vy
    #         + 2 * b * d * py * vx**2
    #         + 2 * b * e * px * vy**2
    #         - 2 * b * e * py * vx * vy
    #         - 4 * b * f * vx * vy
    #         - 4 * c * d * px * vy**2
    #         + 4 * c * d * py * vx * vy
    #         - 4 * c * f * vy**2
    #         + d**2 * vx**2
    #         + 2 * d * e * vx * vy
    #         + e**2 * vy**2
    #     )
    # ) / (2 * (a * vx**2 + b * vx * vy + c * vy**2))

    # (-d*px - e*py - f)/(d*vx + e*vy)


if __name__ == "__main__":
    main()
