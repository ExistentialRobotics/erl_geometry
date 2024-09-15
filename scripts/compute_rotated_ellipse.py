import sympy as sp


def main():
    a, b, cx, cy, theta = sp.symbols("a b cx cy theta")
    px, py, vx, vy = sp.symbols("px py vx vy")

    rotation = sp.Matrix(
        [
            [sp.cos(theta), -sp.sin(theta)],
            [sp.sin(theta), sp.cos(theta)],
        ]
    )
    p = rotation.transpose() @ sp.Matrix([[px - cx], [py - cy]])
    v = rotation.transpose() @ sp.Matrix([[vx], [vy]])

    rx_sq = a ** 2
    ry_sq = b ** 2

    x_diff_sq = v[0] ** 2
    y_diff_sq = v[1] ** 2

    cross_term = p[0] * (p[1] + v[1]) - (p[0] + v[0]) * p[1]
    cross_term_sq = cross_term ** 2

    tmp0 = rx_sq * y_diff_sq + ry_sq * x_diff_sq
    tmp1 = tmp0 - cross_term_sq

    intersected_soft = sp.simplify(tmp1)
    outside_soft = p[0] ** 2 / rx_sq + p[1] ** 2 / ry_sq - 1

    tmp2 = rx_sq * p[1] * v[1] + ry_sq * p[0] * v[0]
    # tmp3 = sp.sqrt(tmp1) * a * b
    # sddf_value = -(tmp3 + tmp2) / tmp0

    t0 = a ** 2 * sp.sin(theta) ** 2 + b ** 2 * sp.cos(theta) ** 2
    t1 = a ** 2 * sp.cos(theta) ** 2 + b ** 2 * sp.sin(theta) ** 2
    t2 = (a ** 2 - b ** 2) * sp.sin(theta) * sp.cos(theta)

    max_order = 0
    max_var_order = 0

    sp.init_printing(wrap_line=False)
    poly_intersected_soft = sp.poly(intersected_soft)
    print("Intersected soft:")
    sp.pretty_print(intersected_soft)
    print(f"Monomials: {len(poly_intersected_soft.monoms())}")
    indices = [
        poly_intersected_soft.gens.index(variable)
        for variable in [px, py, vx, vy]
        if variable in poly_intersected_soft.gens
    ]
    for mono in poly_intersected_soft.monoms():
        term = sp.prod(x ** k for x, k in zip(poly_intersected_soft.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order
    left_vec = sp.Matrix([px ** 2, px * py, py ** 2, px, py, 1]).transpose()
    right_vec = sp.Matrix([vx ** 2, vx * vy, vy ** 2, vx, vy, 1])
    mat = sp.Matrix(
        [
            [0, 0, -1, 0, 0, 0],
            [0, 2, 0, 0, 0, 0],
            [-1, 0, 0, 0, 0, 0],
            [0, -2 * cy, 2 * cx, 0, 0, 0],
            [2 * cy, -2 * cx, 0, 0, 0, 0],
            [
                t0 - cy ** 2,
                -2 * t2 + 2 * cx * cy,
                t1 - cx ** 2,
                0,
                0,
                0,
            ],
        ]
    )
    intersected = (left_vec @ mat @ right_vec)[0]
    print("Intersected:")
    sp.pretty_print(intersected)
    print("Diff:")
    diff = sp.simplify(intersected - intersected_soft)
    sp.pretty_print(diff)

    poly_outside_soft = sp.poly(outside_soft)
    print("Outside soft:")
    sp.pretty_print(outside_soft)
    print(f"Monomials: {len(poly_outside_soft.monoms())}")
    indices = [
        poly_outside_soft.gens.index(variable) for variable in [px, py, vx, vy] if variable in poly_outside_soft.gens
    ]
    for mono in poly_outside_soft.monoms():
        term = sp.prod(x ** k for x, k in zip(poly_outside_soft.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order
    left_vec = sp.Matrix([px ** 2, px * py, py ** 2, px, py, 1])
    right_vec = sp.Matrix(
        [
            t0,
            -2 * t2,
            t1,
            -2 * cx * t0 + 2 * cy * t2,
            -2 * cy * t1 + 2 * cx * t2,
            (cx ** 2 * t0 + cy ** 2 * t1 - 2 * cx * cy * t2) - a ** 2 * b ** 2,
        ]
    ) / (a ** 2 * b ** 2)
    outside = left_vec.dot(right_vec)
    print("Outside:")
    sp.pretty_print(outside)
    print("Diff:")
    diff = sp.simplify(outside - outside_soft)
    sp.pretty_print(diff)

    poly_tmp0 = sp.poly(tmp0)
    print("tmp0:")
    sp.pretty_print(tmp0)
    print(f"Monomials: {len(poly_tmp0.monoms())}")
    indices = [poly_tmp0.gens.index(variable) for variable in [px, py, vx, vy] if variable in poly_tmp0.gens]
    for mono in poly_tmp0.monoms():
        term = sp.prod(x ** k for x, k in zip(poly_tmp0.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order
    left_vec = sp.Matrix([vx ** 2, vx * vy, vy ** 2])
    right_vec = sp.Matrix([a ** 2 * sp.sin(theta) ** 2 + b ** 2 * sp.cos(theta) ** 2,
                           -2 * (a ** 2 - b ** 2) * sp.sin(theta) * sp.cos(theta),
                           a ** 2 * sp.cos(theta) ** 2 + b ** 2 * sp.sin(theta) ** 2])
    tmp0_expr = left_vec.dot(right_vec)
    print("Tmp0:")
    sp.pretty_print(tmp0_expr)
    print("Diff:")
    diff = sp.simplify(tmp0_expr - tmp0)
    sp.pretty_print(diff)

    poly_tmp2 = sp.poly(tmp2)
    print("tmp2:")
    sp.pretty_print(tmp2)
    print(f"Monomials: {len(poly_tmp2.monoms())}")
    indices = [poly_tmp2.gens.index(variable) for variable in [px, py, vx, vy] if variable in poly_tmp2.gens]
    for mono in poly_tmp2.monoms():
        term = sp.prod(x ** k for x, k in zip(poly_tmp2.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order
    left_vec = sp.Matrix([px, py, 1])
    right_vec = sp.Matrix([vx, vy])
    mat = sp.Matrix([
        [t0, -t2],
        [-t2, t1],
        [-cx * t0 + cy * t2, cx * t2 - cy * t1]
    ])
    tmp2_expr = (left_vec.transpose() @ mat @ right_vec)[0]
    print("Tmp2:")
    sp.pretty_print(tmp2_expr)
    print("Diff:")
    diff = sp.simplify(tmp2_expr - tmp2)
    sp.pretty_print(diff)

    total_cnt_monomials = (
            len(poly_intersected_soft.monoms())
            + len(poly_outside_soft.monoms())
            + len(poly_tmp0.monoms())
            + len(poly_tmp2.monoms())
    )
    print(f"Total monomials: {total_cnt_monomials}")
    print(f"Max order: {max_order}")
    print(f"Max var order: {max_var_order}")

    sddf_value = -(sp.sqrt(tmp1) * a * b + tmp2) / tmp0
    d_sddf_dx = sp.diff(sddf_value, px)
    d_sddf_dy = sp.diff(sddf_value, py)
    res = d_sddf_dx * vx + d_sddf_dy * vy
    res = sp.simplify(res)
    print("grad_x.T @ v:", res)

    # d_sddf_dvx = sp.diff(sddf_value, vx)
    # d_sddf_dvy = sp.diff(sddf_value, vy)
    # res = d_sddf_dvx * vx + d_sddf_dvy * vy
    # res = sp.simplify(res)
    # print("grad_v.T @ v:", res)


if __name__ == "__main__":
    main()
