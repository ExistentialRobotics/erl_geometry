import sympy as sp


def intersection_line_line_2d():
    print("Intersection of two lines.")
    x00, y00, x01, y01 = sp.symbols("x00 y00 x01 y01")  # line 1
    x10, y10, x11, y11 = sp.symbols("x10 y10 x11 y11")  # line 2

    lam1 = sp.symbols("lam1")
    lam2 = sp.symbols("lam2")

    intersection = sp.solve(
        [
            x00 + lam1 * (x01 - x00) - x10 - lam2 * (x11 - x10),
            y00 + lam1 * (y01 - y00) - y10 - lam2 * (y11 - y10),
        ]
    )
    intersection = [sp.simplify(x) for x in intersection]
    sp.pretty_print(intersection, num_columns=1000)

    sp.pretty_print(
        [
            sp.simplify(x00 + intersection[0][lam1] * (x01 - x00) - x10 - intersection[0][lam2] * (x11 - x10)),
            sp.simplify(y00 + intersection[0][lam1] * (y01 - y00) - y10 - intersection[0][lam2] * (y11 - y10)),
        ]
    )


def intersection_ray_line_2d():
    print("Intersection of a ray and a line.")
    x0, y0, vx, vy = sp.symbols("x0 y0 vx vy")  # ray
    x1, y1, x2, y2 = sp.symbols("x1 y1 x2 y2")  # line

    lam = sp.symbols("lam")
    dist = sp.symbols("dist")

    intersection = sp.solve(
        [
            x0 + dist * vx - x1 - lam * (x2 - x1),
            y0 + dist * vy - y1 - lam * (y2 - y1),
        ]
    )
    intersection = [sp.simplify(x) for x in intersection]
    sp.pretty_print(intersection, num_columns=1000)


def intersection_line_ellipse():
    print("Intersection of a line and an ellipse: assume the ellipse is centered at the origin.")

    x0, y0, x1, y1 = sp.symbols("x0 y0 x1 y1")  # line
    a, b = sp.symbols("a b")  # ellipse

    # line
    lam = sp.symbols("lam")
    x = x0 + lam * (x1 - x0)
    y = y0 + lam * (y1 - y0)

    # ellipse
    ellipse = (x / a) ** 2 + (y / b) ** 2 - 1

    # intersection
    intersection = sp.solve([ellipse], [lam])
    intersection = [sp.simplify(x) for x in intersection]
    sp.pretty_print(intersection, num_columns=1000)


def intersection_line_ellipsoid():
    print("Intersection of a line and an ellipsoid: assume the ellipsoid is centered at the origin.")

    x0, y0, z0, x1, y1, z1 = sp.symbols("x0 y0 z0 x1 y1 z1")  # line
    a, b, c = sp.symbols("a b c")  # ellipsoid

    # line
    lam = sp.symbols("lam")
    x = x0 + lam * (x1 - x0)
    y = y0 + lam * (y1 - y0)
    z = z0 + lam * (z1 - z0)

    # ellipsoid
    ellipsoid = (x / a) ** 2 + (y / b) ** 2 + (z / c) ** 2 - 1

    # intersection
    intersection = sp.solve(ellipsoid, [lam])
    # intersection = [sp.simplify(x) for x in intersection]
    for x in intersection:
        sp.pretty_print(x, num_columns=1000)


def intersection_line_ray_2d():
    a, b, c = sp.symbols("a b c")  # line
    x0, y0, vx, vy = sp.symbols("x0 y0 vx vy")  # ray

    lam = sp.symbols("lam")
    x = x0 + lam * vx
    y = y0 + lam * vy
    intersection = sp.solve(a * x + b * y + c, [lam])
    intersection = [sp.simplify(x) for x in intersection]
    sp.pretty_print(intersection, num_columns=1000)


def convert_line():
    x0, y0, nx, ny = sp.symbols("x0 y0 nx ny")
    x1, y1 = sp.symbols("x1 y1")

    # cross
    cross = nx * (y1 - y0) - ny * (x1 - x0)
    sol = sp.solve(cross - 1, [x1, y1])
    sp.pretty_print(sol, num_columns=1000)


def main():
    # intersection_line_line_2d()
    # intersection_ray_line_2d()
    # intersection_line_ellipse()
    # intersection_line_ellipsoid()
    # intersection_line_ray_2d()
    convert_line()


if __name__ == "__main__":
    main()
