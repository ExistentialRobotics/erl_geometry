import sympy as sp


def convert_poly_to_matrix(poly, row_symbols, col_symbols, degree):
    # create x**degree, x**(degree-1)y, ..., x**0
    row_vars = sp.poly(sum(row_symbols, sp.Integer(1)) ** sp.Integer(degree))
    row_vars = [sp.prod(x**k for x, k in zip(row_vars.gens, mono)) for mono in row_vars.monoms()]
    row_vars = sorted(row_vars, key=lambda x: 0 if x == 1 else sp.poly(x).total_degree(), reverse=True)

    col_vars = sp.poly(sum(col_symbols, sp.Integer(1)) ** sp.Integer(degree))
    col_vars = [sp.prod(x**k for x, k in zip(col_vars.gens, mono)) for mono in col_vars.monoms()]
    col_vars = sorted(col_vars, key=lambda x: 0 if x == 1 else sp.poly(x).total_degree(), reverse=True)

    mat = sp.zeros(len(row_vars), len(col_vars))
    variables = []
    for i, row_var in enumerate(row_vars):
        for j, col_var in enumerate(col_vars):
            if row_var == 1 and col_var == 1:
                continue
            variables.append((i, j, (row_var * col_var).as_expr()))
    variables = sorted(variables, key=lambda x: sp.poly(x[2]).total_degree(), reverse=True)

    remaining = sp.expand(poly).as_expr()
    for i, j, variable in variables:
        variable = variable.as_expr()
        coeff = sp.expand(remaining).coeff(variable)
        coeff = coeff.as_expr()
        coeff = sp.simplify(coeff)
        if coeff == 0:
            print(f"{variable}: {coeff}")
            continue
        remaining = remaining - coeff * variable
        remaining = sp.simplify(remaining)
        mat[i, j] = coeff
        print(f"{variable}: {coeff}")
        # sp.pretty_print(coeff)
        print()

    mat[-1, -1] = sp.simplify(remaining)
    print(f"Remaining: {remaining}")
    return mat


def main():
    # a, b, c, cx, cy, cz, theta, phi = sp.symbols("a b c cx cy cz theta phi")
    a, b, c, cx, cy, cz = sp.symbols("a b c cx cy cz")
    px, py, pz, vx, vy, vz = sp.symbols("px py pz vx vy vz")

    # theta: azimuthal angle
    # phi: elevation angle
    # rotation_z = sp.Matrix(
    #     [
    #         [sp.cos(theta), -sp.sin(theta), 0],
    #         [sp.sin(theta), sp.cos(theta), 0],
    #         [0, 0, 1],
    #     ]
    # )
    # rotation_y = sp.Matrix(  # rotate around y-axis by -phi
    #     [
    #         [sp.cos(phi), 0, -sp.sin(phi)],
    #         [0, 1, 0],
    #         [sp.sin(phi), 0, sp.cos(phi)],
    #     ]
    # )
    # rotation = rotation_z @ rotation_y
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = sp.symbols("r11 r12 r13 r21 r22 r23 r31 r32 r33")
    rotation = sp.Matrix(
        [
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33],
        ]
    )
    # rotation = sp.MatrixSymbol("R", 3, 3)

    # print("Rotation:")
    # sp.pretty_print(rotation)
    # d_rotation_d_theta = sp.diff(rotation, theta)
    # print("d_rotation_d_theta:")
    # sp.pretty_print(d_rotation_d_theta)
    # d_rotation_d_phi = sp.diff(rotation, phi)
    # print("d_rotation_d_phi:")
    # sp.pretty_print(d_rotation_d_phi)

    p = rotation.transpose() @ sp.Matrix([[px - cx], [py - cy], [pz - cz]])
    v = rotation.transpose() @ sp.Matrix([[vx], [vy], [vz]])

    print("p:")
    sp.pretty_print(p)
    dp_dcx = sp.diff(p, cx)
    print("dp_dcx:")
    sp.pretty_print(dp_dcx)
    dp_dcy = sp.diff(p, cy)
    print("dp_dcy:")
    sp.pretty_print(dp_dcy)
    dp_dcz = sp.diff(p, cz)
    print("dp_dcz:")
    sp.pretty_print(dp_dcz)

    print("v:")
    sp.pretty_print(v)

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

    intersected_soft = tmp1  # sp.simplify(tmp1)
    sign_soft = p[0] ** 2 / rx_sq + p[1] ** 2 / ry_sq + p[2] ** 2 / rz_sq - 1

    tmp2 = ry_sq_rz_sq * p[0] * v[0] + rz_sq_rx_sq * p[1] * v[1] + rx_sq_ry_sq * p[2] * v[2]
    tmp3 = sp.sqrt(tmp1) * a * b * c
    sddf_value = -(tmp3 + tmp2) / tmp0

    d_tmp0_d_px = sp.diff(tmp0, px)
    d_tmp0_d_py = sp.diff(tmp0, py)
    d_tmp0_d_pz = sp.diff(tmp0, pz)
    grad_p_tmp0_v = d_tmp0_d_px * vx + d_tmp0_d_py * vy + d_tmp0_d_pz * vz
    grad_p_tmp0_v = sp.simplify(grad_p_tmp0_v)
    print("grad_tmp0_p:")
    sp.pretty_print(grad_p_tmp0_v)

    d_tmp1_d_px = sp.diff(tmp1, px)
    d_tmp1_d_py = sp.diff(tmp1, py)
    d_tmp1_d_pz = sp.diff(tmp1, pz)
    grad_p_tmp1_v = d_tmp1_d_px * vx + d_tmp1_d_py * vy + d_tmp1_d_pz * vz
    grad_p_tmp1_v = sp.simplify(grad_p_tmp1_v)
    print("grad_tmp1_p:")
    sp.pretty_print(grad_p_tmp1_v)

    d_tmp2_d_px = sp.diff(tmp2, px)
    d_tmp2_d_py = sp.diff(tmp2, py)
    d_tmp2_d_pz = sp.diff(tmp2, pz)
    grad_p_tmp2_v = d_tmp2_d_px * vx + d_tmp2_d_py * vy + d_tmp2_d_pz * vz
    grad_p_tmp2_v = sp.simplify(grad_p_tmp2_v)
    print("grad_tmp2_p:")
    sp.pretty_print(grad_p_tmp2_v)
    grad_sddf_p_v = -grad_p_tmp2_v / tmp0
    grad_sddf_p_v = sp.simplify(grad_sddf_p_v)
    print("grad_sddf_p:")
    sp.pretty_print(grad_sddf_p_v)

    # too slow
    # d_sddf_dpx = sp.diff(sddf_value, px)
    # d_sddf_dpy = sp.diff(sddf_value, py)
    # d_sddf_dpz = sp.diff(sddf_value, pz)
    # grad_p_sddf_v = d_sddf_dpx * vx + d_sddf_dpy * vy + d_sddf_dpz * vz
    # grad_p_sddf_v = sp.simplify(grad_p_sddf_v)
    # print("grad_sddf_p:")
    # sp.pretty_print(grad_p_sddf_v)

    # d_i_vx = sp.diff(intersected_soft, vx)
    # d_i_vy = sp.diff(intersected_soft, vy)
    # d_i_vz = sp.diff(intersected_soft, vz)
    # grad_v_i_v = d_i_vx * vx + d_i_vy * vy + d_i_vz * vz
    # grad_v_i_v_2 = 2 * intersected_soft
    # diff = sp.simplify(grad_v_i_v - grad_v_i_v_2)
    # print("diff:")  # =0
    # sp.pretty_print(diff)

    # d_tmp0_vx = sp.diff(tmp0, vx)
    # d_tmp0_vy = sp.diff(tmp0, vy)
    # d_tmp0_vz = sp.diff(tmp0, vz)
    # grad_v_tmp0_v = d_tmp0_vx * vx + d_tmp0_vy * vy + d_tmp0_vz * vz
    # grad_v_tmp0_v_2 = 2 * tmp0
    # diff = sp.simplify(grad_v_tmp0_v - grad_v_tmp0_v_2)
    # print("diff:")  # =0
    # sp.pretty_print(diff)

    # d_tmp2_vx = sp.diff(tmp2, vx)
    # d_tmp2_vy = sp.diff(tmp2, vy)
    # d_tmp2_vz = sp.diff(tmp2, vz)
    # grad_v_tmp2_v = d_tmp2_vx * vx + d_tmp2_vy * vy + d_tmp2_vz * vz
    # grad_v_tmp2_v_2 = tmp2
    # diff = sp.simplify(grad_v_tmp2_v - grad_v_tmp2_v_2)
    # print("diff:") # =0
    # sp.pretty_print(diff)

    d_sddf_dvx = sp.diff(sddf_value, vx)
    d_sddf_dvy = sp.diff(sddf_value, vy)
    d_sddf_dvz = sp.diff(sddf_value, vz)
    grad_v_sddf_v = d_sddf_dvx * vx + d_sddf_dvy * vy + d_sddf_dvz * vz
    diff = sp.simplify(grad_v_sddf_v - 3 * sddf_value)
    print("diff:")
    sp.pretty_print(diff)
    # grad_v_sddf_v = sp.simplify(grad_v_sddf_v)
    # print("grad_sddf_v:")
    # sp.pretty_print(grad_v_sddf_v)
    return

    max_order = 0
    max_var_order = 0

    sp.init_printing(wrap_line=False)

    print("Intersected soft:")
    sp.pretty_print(intersected_soft)

    # d_intersected_soft_d_a = sp.diff(intersected_soft, a)
    # print("Derivative w.r.t. a:")
    # sp.pretty_print(sp.simplify(d_intersected_soft_d_a))

    # d_intersected_soft_d_b = sp.diff(intersected_soft, b)
    # print("Derivative w.r.t. b:")
    # sp.pretty_print(sp.simplify(d_intersected_soft_d_b))

    # d_intersected_soft_d_c = sp.diff(intersected_soft, c)
    # print("Derivative w.r.t. c:")
    # sp.pretty_print(sp.simplify(d_intersected_soft_d_c))

    # d_intersected_soft_d_cx = sp.diff(intersected_soft, cx)
    # print("Derivative w.r.t. cx:")
    # sp.pretty_print(sp.simplify(d_intersected_soft_d_cx))
    # return

    # convert to matrix form: [px, py, pz, 1] * mat * [vx, vy, vz, 1]
    # convert_poly_to_matrix(intersected_soft, [px, py, pz], [vx, vy, vz], 2)

    poly_intersected_soft = sp.poly(intersected_soft)
    print(f"Monomials: {len(poly_intersected_soft.monoms())}")
    # find the index of px, py, pz, vx, vy, vz in gens
    indices = [
        poly_intersected_soft.gens.index(variable)
        for variable in [px, py, pz, vx, vy, vz]
        if variable in poly_intersected_soft.gens
    ]
    for i, mono in enumerate(poly_intersected_soft.monoms()):
        term = sp.prod(x**k for x, k in zip(poly_intersected_soft.gens, mono))
        sp.pretty_print(term)
        order = sum(mono[index] for index in indices)
        if order > max_order:
            max_order = order
        order = max(mono[index] for index in indices)
        if order > max_var_order:
            max_var_order = order

    poly_sign_soft = sp.poly(sign_soft)
    print("Outside soft:")
    sp.pretty_print(sign_soft)
    print(f"Monomials: {len(poly_sign_soft.monoms())}")
    indices = [
        poly_sign_soft.gens.index(variable) for variable in [px, py, pz, vx, vy, vz] if variable in poly_sign_soft.gens
    ]
    for mono in poly_sign_soft.monoms():
        term = sp.prod(x**k for x, k in zip(poly_sign_soft.gens, mono))
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
        + len(poly_sign_soft.monoms())
        + len(poly_tmp0.monoms())
        + len(poly_tmp2.monoms())
    )

    print(f"Total monomials: {total_cnt_monomials}")
    print(f"Max order: {max_order}")
    print(f"Max var order: {max_var_order}")


if __name__ == "__main__":
    main()
