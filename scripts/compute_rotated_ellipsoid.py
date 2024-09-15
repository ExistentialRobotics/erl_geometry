import sympy as sp


def main():
    a, b, c, cx, cy, cz, theta, phi = sp.symbols("a b c cx cy cz theta phi")
    px, py, pz, vx, vy, vz = sp.symbols("px py pz vx vy vz")

    # theta: azimuthal angle
    # phi: elevation angle
    rotation_z = sp.Matrix(
        [
            [sp.cos(theta), -sp.sin(theta), 0],
            [sp.sin(theta), sp.cos(theta), 0],
            [0, 0, 1],
        ]
    )
    rotation_y = sp.Matrix(
        [
            [sp.cos(phi), 0, sp.sin(phi)],
            [0, 1, 0],
            [-sp.sin(phi), 0, sp.cos(phi)],
        ]
    )
    rotation = rotation_z @ rotation_y

    p = rotation @ sp.Matrix([[px - cx], [py - cy], [pz - cz]])
    v = rotation @ sp.Matrix([[vx], [vy], [vz]])

    rx_sq = a**2
    ry_sq = b**2
    rz_sq = c**2

    rx_sq_ry_sq = rx_sq * ry_sq
    ry_sq_rz_sq = ry_sq * rz_sq
    rz_sq_rx_sq = rz_sq * rx_sq

    x_diff_sq = v[0] ** 2
    y_diff_sq = v[1] ** 2
    z_diff_sq = v[2] ** 2

    cross_x = p[1] * v[2] - p[2] * v[1]
    cross_y = p[2] * v[0] - p[0] * v[2]
    cross_z = p[0] * v[1] - p[1] * v[0]
    cross_term_sq = rx_sq * cross_x**2 + ry_sq * cross_y**2 + rz_sq * cross_z**2

    tmp0 = ry_sq_rz_sq * x_diff_sq + rz_sq_rx_sq * y_diff_sq + rx_sq_ry_sq * z_diff_sq
    tmp1 = tmp0 - cross_term_sq

    intersected_soft = sp.simplify(tmp1)
    outside_soft = p[0] ** 2 / rx_sq + p[1] ** 2 / ry_sq + p[2] ** 2 / rz_sq - 1

    tmp2 = ry_sq_rz_sq * p[0] * v[0] + rz_sq_rx_sq * p[1] * v[1] + rx_sq_ry_sq * p[2] * v[2]
    # tmp3 = sp.sqrt(tmp1) * a * b * c
    # sddf_value = -(tmp3 + tmp2) / tmp0

    max_order = 0
    max_var_order = 0

    sp.init_printing(wrap_line=False)
    poly_intersected_soft = sp.poly(intersected_soft)
    print("Intersected soft:")
    sp.pretty_print(intersected_soft)
    print(f"Monomials: {len(poly_intersected_soft.monoms())}")
    # find the index of px, py, pz, vx, vy, vz in gens
    indices = [
        poly_intersected_soft.gens.index(variable)
        for variable in [px, py, pz, vx, vy, vz]
        if variable in poly_intersected_soft.gens
    ]
    for mono in poly_intersected_soft.monoms():
        term = sp.prod(x**k for x, k in zip(poly_intersected_soft.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order

    poly_outside_soft = sp.poly(outside_soft)
    print("Outside soft:")
    sp.pretty_print(outside_soft)
    print(f"Monomials: {len(poly_outside_soft.monoms())}")
    indices = [
        poly_outside_soft.gens.index(variable)
        for variable in [px, py, pz, vx, vy, vz]
        if variable in poly_outside_soft.gens
    ]
    for mono in poly_outside_soft.monoms():
        term = sp.prod(x**k for x, k in zip(poly_outside_soft.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order

    poly_tmp0 = sp.poly(tmp0)
    print("tmp0:")
    sp.pretty_print(tmp0)
    print(f"Monomials: {len(poly_tmp0.monoms())}")
    indices = [poly_tmp0.gens.index(variable) for variable in [px, py, pz, vx, vy, vz] if variable in poly_tmp0.gens]
    for mono in poly_tmp0.monoms():
        term = sp.prod(x**k for x, k in zip(poly_tmp0.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order

    poly_tmp2 = sp.poly(tmp2)
    print("tmp2:")
    sp.pretty_print(tmp2)
    print(f"Monomials: {len(poly_tmp2.monoms())}")
    indices = [poly_tmp2.gens.index(variable) for variable in [px, py, pz, vx, vy, vz] if variable in poly_tmp2.gens]
    for mono in poly_tmp2.monoms():
        term = sp.prod(x**k for x, k in zip(poly_tmp2.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order

    total_cnt_monomials = (
        len(poly_intersected_soft.monoms())
        + len(poly_outside_soft.monoms())
        + len(poly_tmp0.monoms())
        + len(poly_tmp2.monoms())
    )

    print(f"Total monomials: {total_cnt_monomials}")
    print(f"Max order: {max_order}")
    print(f"Max var order: {max_var_order}")


if __name__ == "__main__":
    main()
