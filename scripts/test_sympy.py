import sympy as sp

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

rx_sq = a**2
ry_sq = b**2

x_diff_sq = v[0] ** 2
y_diff_sq = v[1] ** 2

cross_term = p[0] * (p[1] + v[1]) - (p[0] + v[0]) * p[1]
cross_term_sq = cross_term**2

tmp0 = rx_sq * y_diff_sq + ry_sq * x_diff_sq
tmp1 = tmp0 - cross_term_sq

# expand tmp1
tmp1_expanded = sp.expand(tmp1)
print(tmp1_expanded)

row_vars = sp.poly(sum([px, py], sp.Integer(1)) ** sp.Integer(2))
row_vars = [sp.prod(x**k for x, k in zip(row_vars.gens, mono)) for mono in row_vars.monoms()]
row_vars = sorted(row_vars, key=lambda x: 0 if x == 1 else sp.poly(x).total_degree(), reverse=True)

col_vars = sp.poly(sum([vx, vy], sp.Integer(1)) ** sp.Integer(2))
col_vars = [sp.prod(x**k for x, k in zip(col_vars.gens, mono)) for mono in col_vars.monoms()]
col_vars = sorted(col_vars, key=lambda x: 0 if x == 1 else sp.poly(x).total_degree(), reverse=True)

variables = []
for i, row_var in enumerate(row_vars):
    for j, col_var in enumerate(col_vars):
        if row_var == 1 and col_var == 1:
            continue
        variables.append((i, j, (row_var * col_var).as_expr()))
variables = sorted(variables, key=lambda x: sp.poly(x[2]).total_degree(), reverse=True)
print(f"variables: {variables}")

# coeff = tmp1_expanded.coeff(vx**2)
# coeff = sp.simplify(coeff)
# print(f"coeff: {coeff}")
# remaining = sp.simplify(tmp1_expanded - coeff * vx**2)
# print(f"remaining: {remaining}")

remaining = tmp1_expanded
for i, j, variable in variables:
    variable = variable.as_expr()
    coeff = sp.expand(remaining).coeff(variable)
    coeff = coeff.as_expr()
    remaining = remaining - coeff * variable
    coeff = sp.simplify(coeff)
    remaining = sp.simplify(remaining)
    print(f"{variable}: {coeff}")
    print(f"remaining: {remaining}")
    print()
