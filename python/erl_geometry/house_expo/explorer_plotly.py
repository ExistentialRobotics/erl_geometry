import argparse
import os
from typing import Dict
from typing import List
from typing import Tuple

import dash
import dash.dcc as dcc
import dash.html as dhc
import numpy as np
import plotly.graph_objects as go
from dash import Input
from dash import Output
from erl_common.storage import GridMapInfo2D

from . import HouseExpoMap
from .list_data import get_map_and_traj_files
from .list_data import get_map_dir
from .list_data import get_map_filenames
from .list_data import get_traj_dir
from .sequence import load_trajectory


def create_div(components: list) -> dhc.Div:
    return dhc.Div(components, style=dict(display="inline-block", width="100%"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-trajectories-only", action="store_true")
    args = parser.parse_args()

    ID_INPUT_ONE_PERCENT = "id-input-one-percent"
    ID_DROPDOWN_JSON = "id-dropdown-json"
    ID_BUTTON_PREVIOUS = "id-button-previous"
    ID_BUTTON_NEXT = "id-button-next"
    ID_GRAPH = "id-graph"
    ID_LABEL_INDEX = "id-label-index"

    if args.with_trajectories_only:
        files = get_map_and_traj_files()
        json_files = [f[0] for f in files]
        traj_files = [f[1] for f in files]
    else:
        json_files = get_map_filenames()
    json_dir = get_map_dir()
    traj_dir = get_traj_dir()
    configs: Dict[str, int] = dict()

    app = dash.Dash("HouseExpoMap Explorer", external_stylesheets=["assets/custom.css"])
    app.layout = dhc.Div(
        [
            create_div(
                [
                    dhc.Label(
                        "Which 1%:",
                        style=dict(
                            fontSize=13,
                            height=20,
                            width="6%",
                            textAlign="right",
                            display="inline-block",
                            verticalAlign="middle",
                        ),
                    ),
                    dcc.Input(
                        id=ID_INPUT_ONE_PERCENT,
                        min=0,
                        max=99,
                        step=1,
                        value=0,
                        type="number",
                        style=dict(
                            fontSize=15,
                            height=20,
                            marginRight="1%",
                            width="5%",
                            display="inline-block",
                            verticalAlign="middle",
                        ),
                    ),
                    dhc.Label(
                        "Which JSON:",
                        style=dict(
                            fontSize=13,
                            height=20,
                            width="7%",
                            textAlign="right",
                            display="inline-block",
                            verticalAlign="middle",
                            marginRight="15px",
                        ),
                    ),
                    dcc.Dropdown(
                        id=ID_DROPDOWN_JSON,
                        style=dict(
                            fontSize=15,
                            height=36,
                            width="60%",
                            display="inline-block",
                            verticalAlign="middle",
                        ),
                    ),
                    dhc.Label("Index: "),
                    dhc.Label(
                        f"{configs.get('index', 0)}",
                        id=ID_LABEL_INDEX,
                        style=dict(
                            display="inline-block",
                            verticalAlign="middle",
                            marginRight="20px",
                        ),
                    ),
                    dhc.Button(
                        "Previous",
                        id=ID_BUTTON_PREVIOUS,
                        style=dict(
                            fontSize=15,
                            height=26,
                            width="8%",
                            display="inline-block",
                            verticalAlign="middle",
                        ),
                    ),
                    dhc.Button(
                        "Next",
                        id=ID_BUTTON_NEXT,
                        style=dict(
                            fontSize=15,
                            height=26,
                            width="8%",
                            display="inline-block",
                            verticalAlign="middle",
                        ),
                    ),
                ]
            ),
            dhc.Hr(),
            dcc.Graph(
                id=ID_GRAPH,
                style=dict(width="1000px", height="1000px", display="inline-block"),
            ),
        ],
    )

    @app.callback(
        [
            Output(ID_GRAPH, "figure"),
            Output(ID_DROPDOWN_JSON, "options"),
            Output(ID_DROPDOWN_JSON, "value"),
            Output(ID_LABEL_INDEX, "children"),
        ],
        [
            Input(ID_INPUT_ONE_PERCENT, "value"),
            Input(ID_DROPDOWN_JSON, "value"),
            Input(ID_BUTTON_PREVIOUS, "n_clicks"),
            Input(ID_BUTTON_NEXT, "n_clicks"),
        ],
    )
    def update_dropdown_json(start_: int, json_filename: str, *_: dict) -> Tuple[List[str], str, str]:
        ctx = dash.callback_context
        output_graph = dash.no_update
        output_dropdown_options = dash.no_update
        output_dropdown_value = dash.no_update
        output_label_index = dash.no_update

        if ctx.triggered_id == ID_BUTTON_PREVIOUS:
            configs["index"] -= 1
            configs["index"] = max(min(configs["index"], len(json_files) - 1), 0)
            output_dropdown_value = json_files[configs["index"]]
            output_label_index = str(configs["index"])
        elif ctx.triggered_id == ID_BUTTON_NEXT:
            configs["index"] += 1
            configs["index"] = max(min(configs["index"], len(json_files) - 1), 0)
            output_dropdown_value = json_files[configs["index"]]
            output_label_index = str(configs["index"])
        elif ctx.triggered_id == ID_DROPDOWN_JSON:
            configs["index"] = json_files.index(json_filename)
            output_label_index = str(configs["index"])
        else:
            start = start_ / 100
            end = start + 0.01
            start = int(round(len(json_files) * start))
            end = int(round(len(json_files) * end))
            configs["start"] = start
            configs["end"] = end
            configs["index"] = start
            output_dropdown_options = json_files[start:end]
            output_dropdown_value = json_files[start]
            output_label_index = str(configs["index"])

        json_filename = json_files[configs["index"]]
        house_expo_map = HouseExpoMap(json_filename)
        token = os.path.splitext(json_filename)[0]
        csv_file = os.path.join(traj_dir, f"{token}.csv")

        width = 1000
        vertices = house_expo_map.meter_space.surface.vertices
        xmin = np.min(vertices[0])
        xmax = np.max(vertices[0])
        ymin = np.min(vertices[1])
        ymax = np.max(vertices[1])
        height = int(np.round(width * (ymax - ymin) / (xmax - xmin)))
        resolutions = np.array([(xmax - xmin) / width, (ymax - ymin) / height])
        padding = np.array([2, 2])
        grid_map_info = GridMapInfo2D(np.array([xmin, ymin]), np.array([xmax, ymax]), resolutions, padding)
        img = house_expo_map.meter_space.generate_map_image(grid_map_info)[::-1]

        data = [
            go.Contour(
                x=grid_map_info.get_dim_lin_space(0),
                y=grid_map_info.get_dim_lin_space(1),
                z=img,
                colorscale="gray",
                showscale=False,
            )
        ]

        if os.path.exists(csv_file):
            path = load_trajectory(csv_file)
            data.append(go.Scatter(x=path[:, 0], y=path[:, 1], name="trajectory"))
            data.append(
                go.Scatter(
                    x=path[0, [0]],
                    y=path[0, [1]],
                    mode="markers",
                    marker=dict(size=10),
                    name="Start",
                )
            )
            data.append(
                go.Scatter(
                    x=path[-1, [0]],
                    y=path[-1, [1]],
                    mode="markers",
                    marker=dict(size=10),
                    name="End",
                )
            )

        output_graph = go.Figure(data=data)
        return output_graph, output_dropdown_options, output_dropdown_value, output_label_index

    app.run(debug=True, port=8050, host="0.0.0.0")


if __name__ == "__main__":
    main()
