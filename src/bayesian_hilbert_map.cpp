#include "erl_geometry/bayesian_hilbert_map.hpp"

YAML::Node
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::encode(
    const erl::geometry::BayesianHilbertMapSetting &setting) {
    Node node;
    ERL_YAML_SAVE_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, max_distance);
    ERL_YAML_SAVE_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_SAVE_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_SAVE_ATTR(node, setting, init_sigma);
    ERL_YAML_SAVE_ATTR(node, setting, num_em_iterations);
    ERL_YAML_SAVE_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_SAVE_ATTR(node, setting, use_sparse);
    return node;
}

bool
YAML::convert<erl::geometry::BayesianHilbertMapSetting>::decode(
    const Node &node,
    erl::geometry::BayesianHilbertMapSetting &setting) {
    if (!node.IsMap()) { return false; }
    ERL_YAML_LOAD_ATTR(node, setting, diagonal_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, max_distance);
    ERL_YAML_LOAD_ATTR(node, setting, free_points_per_meter);
    ERL_YAML_LOAD_ATTR(node, setting, free_sampling_margin);
    ERL_YAML_LOAD_ATTR(node, setting, init_sigma);
    ERL_YAML_LOAD_ATTR(node, setting, num_em_iterations);
    ERL_YAML_LOAD_ATTR(node, setting, sparse_zero_threshold);
    ERL_YAML_LOAD_ATTR(node, setting, use_sparse);
    return true;
}
