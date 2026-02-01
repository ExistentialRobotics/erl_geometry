# Bayesian Hilbert Map

## Hilbert Maps

Hilbert maps are a type of continuous occupancy mapping technique that uses kernel methods to represent the environment.
They are based on an approximate kernel defined by the inner
product $\operatorname{kern}(\mathbf{x},\tilde{\mathbf{x}})\approx\Phi(\mathbf{x})^\top \Phi(\tilde{\mathbf{x}})$.
They are particularly useful in robotics for tasks such as navigation and mapping, as they can efficiently handle
large-scale environments and provide smooth occupancy estimates.

This note focuses on using hinged features defined by the RBF kernel:
$$k(\mathbf{x},\tilde{\mathbf{x}}) = \exp\left(-\frac{\|\mathbf{x}-\tilde{\mathbf{x}}\|_2^2}{2l^2}\right),$$
where $\Sigma$ is a diagonal matrix defining the length-scales of the kernel.

Having $D$ hinged points $\{\tilde{\mathbf{x}}_i\}_{i=1}^D$,
the feature vector is defined as:
$$\Phi(\mathbf{x}) = \left[\begin{array}{c}
1 &
k(\mathbf{x},\tilde{\mathbf{x}}_1) &
k(\mathbf{x},\tilde{\mathbf{x}}_2) &
\cdots &
k(\mathbf{x},\tilde{\mathbf{x}}_D)
\end{array}\right]^\top.$$

The probability that a point $\mathbf{x}$ has occupancy $y \in \{-1,1\}$ is modeled using logistic regression:
$$p(y|\mathbf{x},\mathbf{w}) = \frac{1}{1+\exp(-y\mathbf{w}^\top \Phi(\mathbf{x}))} := \sigma(-y\mathbf{w}^\top \Phi(\mathbf{x})), $$
where $\mathbf{w} \in \mathbb{R}^{D+1}$ are the weights of the model, $y \in \{-1, 1\}$.

## Bayesian Hilbert Maps

Given a dataset $\mathcal{D}_t = \{(\mathbf{x}_k,y_k)\}_{k=1}^{K_t}$ of $K_t$ observations at time $t$, our goal is to
update the weights $\mathbf{w}$ of the model such that we get the posterior distribution:
$$p(\mathbf{w}|\mathcal{D}_t) = \frac{p(\mathcal{D}_t|\mathbf{w})p(\mathbf{w})}{p(\mathcal{D}_t)},$$
where $p(\mathbf{w})$ is the prior distribution of the weights, and the likelihood is given by:
$$p(\mathcal{D}_t|\mathbf{w}) = \prod_{k=1}^{K_t} p(y_k|\mathbf{x}_k,\mathbf{w}) = \prod_{k=1}^{K_t} \sigma(-y_k\mathbf{w}^\top \Phi(\mathbf{x}_k)), y_k \in \{-1, 1\}.$$

The marginal likelihood $p(\mathcal{D}_t)$ is intractable, so we resort to approximate inference methods.
Our goal to maximize the log marginal likelihood:
$$
\begin{aligned}
\log p(\mathcal{D}_t) &= \log \int p(\mathcal{D}_t|\mathbf{w})p(\mathbf{w}) d\mathbf{w} \\
&= \log \int Q(\mathbf{w}) \frac{p(\mathcal{D}_t|\mathbf{w})p(\mathbf{w})}{Q(\mathbf{w})} d\mathbf{w} \\
&\geq \int Q(\mathbf{w}) \log \frac{p(\mathcal{D}_t|\mathbf{w})p(\mathbf{w})}{Q(\mathbf{w})} d\mathbf{w}, \quad \leftarrow \text{Jensen's inequality}
\end{aligned}
$$
where $Q(\mathbf{w})$ is an approximate posterior distribution of the weights.

Since $p(\mathcal{D}_t|\mathbf{w})$ is product of sigmoids that contains the weights $\mathbf{w}$ inside,
we use the variational bound proposed by Jaakkola and Jordan (1997) to get a looser lower bound:
$$\sigma(r) \geq \sigma(\xi) \exp\left(\frac{r - \xi}{2} +\lambda\left(\xi\right) \left(r^2-\xi^2\right) \right),$$
where $\xi$ is a variational parameter and $\lambda(\xi) = \frac{1}{2\xi}\left(\frac{1}{2}-\sigma(\xi)\right)$.

For your interest, [sigmoid_lowerbound.py](sigmoid_lowerbound.py) provides interactive visualization of the
variational lower bound of the sigmoid function.

Therefore, with the assumption of a Gaussian
prior $p(\mathbf{w}) = \mathcal{N}(\mathbf{w}|\mu_{t-1},\Sigma_{t-1})$ and a Gaussian approximate posterior
$Q(\mathbf{w}) = \mathcal{N}(\mathbf{w}|\mu_t,\Sigma_t)$, we can write the variational lower bound of the log marginal
likelihood as:
$$
\begin{aligned}
\mathcal{L} &= \mathbb{E}_{Q(\mathbf{w})} \left[\log p(\mathcal{D}_t|\mathbf{w})\right] + \mathbb{E}_{Q(\mathbf{w})} \left[\log p(\mathbf{w})\right] - \mathbb{E}_{Q(\mathbf{w})} \left[\log Q(\mathbf{w})\right] \\
&\geq \sum_{k=1}^{K_t} \mathbb{E}_{Q(\mathbf{w})} \left[\log \sigma(-y_k\mathbf{w}^\top \Phi(\mathbf{x}_k))\right] + \mathbb{E}_{Q(\mathbf{w})} \left[\log p(\mathbf{w})\right] - \mathbb{E}_{Q(\mathbf{w})} \left[\log Q(\mathbf{w})\right] \\
&\geq \sum_{k=1}^{K_t} \left( \log \sigma(\xi_k) - \frac{\xi_k}{2} + \mathbb{E}_{Q(\mathbf{w})}\left[\frac{r_k}{2}\right] + \lambda(\xi_k) \left( \mathbb{E}_{Q(\mathbf{w})} \left[ r_k^2 \right] - \xi_k^2 \right) \right) + \mathbb{E}_{Q(\mathbf{w})} \left[\log p(\mathbf{w})\right] - \mathbb{E}_{Q(\mathbf{w})} \left[\log Q(\mathbf{w})\right] \\
&= \sum_{k=1}^{K_t} \left( \log \sigma(\xi_k) - \frac{\xi_k}{2} - \xi_k^2\lambda(\xi_k) + (y_k - \frac{1}{2}) \mu_t^\top \Phi(\mathbf{x}_k) + \lambda(\xi_k) \Phi^\top(\mathbf{x}_k) \left( \Sigma_t + \mu_t\mu_t^\top\right) \Phi(\mathbf{x}_k) \right) + \frac{1}{2}\log \frac{|\Sigma_t|}{|\Sigma_{t-1}|} \\
&\quad + \frac{1}{2} \left( \operatorname{tr}\left(\left(\Sigma_{t}^{-1}-\Sigma_{t-1}^{-1}\right)\Sigma_t\right) - \left(\mu_t - \mu_{t-1}\right)^\top \Sigma_{t-1}^{-1} \left(\mu_t - \mu_{t-1}\right) \right),
\end{aligned}$$
where $r_k = y_k\mathbf{w}^\top \Phi(\mathbf{x}_k), y_k \in \{0, 1\}.$

To maximize the lower bound $\mathcal{L}$, we take derivatives with respect to the variational
parameters $\{\xi_k\}_{k=1}^{K_t}$, $\mu_t$, and $\Sigma_t$, and set them to zero.
This results in the following update equations:

- **E-Step**: $\partial \mathcal{L} / \partial \mu_t = 0$ leads to

$$
\begin{aligned}
\mu_t &= \Sigma_t \left( \Sigma_{t-1}^{-1} \mu_{t-1} + \sum_{k=1}^{K_t} \left(y_k - \frac{1}{2}\right) \Phi(\mathbf{x}_k) \right), \\
\Sigma_t^{-1} &= \Sigma_{t-1}^{-1} + 2 \sum_{k=1}^{K_t} \lambda(\xi_k) \Phi(\mathbf{x}_k) \Phi^\top(\mathbf{x}_k).
\end{aligned}
$$

- **M-Step**: $\partial \mathcal{L} / \partial \xi_k = 0$ leads to

$$\xi_k = \sqrt{\Phi^\top(\mathbf{x}_k) \left( \Sigma_t + \mu_t\mu_t^\top \right) \Phi(\mathbf{x}_k)}, \quad \forall k = 1, \ldots, K_t.$$

Note that $\xi_{t,k}=\xi_k$, where $t$ is omitted for simplicity. We suggest initialize $\xi_{0,k}$ to 0.0 or 1.0 for
all $k$ (Empirically, both work well).

## References

1. Tommi S. Jaakkola, Michael I. Jordan. A variational approach to Bayesian logistic regression models and their
   extensions. Proceedings of the Sixth International Workshop on Artificial Intelligence and Statistics, PMLR R1:
   283-294, 1997.
2. Ransalu Senanayake, Fabio Ramos. Bayesian Hilbert Maps for Dynamic Continuous Occupancy Mapping. Proceedings of the
   1st Annual Conference on Robot Learning, PMLR 78:458-471, 2017.
