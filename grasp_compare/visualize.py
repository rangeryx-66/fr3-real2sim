"""Small browser preview of one common scene and all provider TCP frames."""

import numpy as np


def save_html(path, scene, by_model, max_per_model=20):
    import plotly.graph_objects as go
    figure = go.Figure()
    points = scene.points_C
    if len(points) > 15000:
        points = points[np.linspace(0, len(points) - 1, 15000, dtype=int)]
    figure.add_trace(go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2],
                                  mode='markers', marker={'size': 1, 'color': '#aaaaaa'},
                                  name='scene'))
    for model, candidates in by_model.items():
        kept = candidates[:max_per_model]
        if not kept:
            continue
        xyz = np.array([c.T_C_TCP[:3, 3] for c in kept])
        figure.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
                                      mode='markers', marker={'size': 5}, name=model,
                                      text=[f'rank={c.rank} score={c.score:.3g}' for c in kept]))
        for c in kept[:5]:
            p = c.T_C_TCP[:3, 3]
            q = p - .04 * c.T_C_TCP[:3, 2]
            figure.add_trace(go.Scatter3d(x=[q[0], p[0]], y=[q[1], p[1]], z=[q[2], p[2]],
                                          mode='lines', line={'width': 4}, showlegend=False,
                                          hoverinfo='skip'))
    figure.update_layout(scene={'aspectmode': 'data'}, title='Common camera frame; lines show approach')
    figure.write_html(str(path), include_plotlyjs='cdn')
