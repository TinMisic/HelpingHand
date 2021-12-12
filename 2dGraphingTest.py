import plotly.graph_objects as go
import numpy as np

x=(1.70940171E-4,1.65975104E-4,1.6E-4,1.54083205E-4,1.48920328E-4,1.42755175E-4,1.37931034E-4,1.33244504E-4,1.28865979E-4,1.23228589E-4,1.19760479E-4,1.14155251E-4,1.10803324E-4,1.06213489E-4)
y=(0,0.00057,0.00114,0.00171,0.00228,0.00285,0.00342,0.00399,0.00457,0.00514,0.00571,0.00629,0.00686,0.00743)
#layout = go.Layout(yaxis=dict(scaleanchor="x", scaleratio=1))

fig = go.Figure(data=go.Scatter(x=x,y=y))
fig.update_layout(
    annotations=[
        dict(
            x=0.5,
            y=-0.15,
            showarrow=False,
            text="x",
            xref="paper",
            yref="paper"
        ),
        dict(
            x=-0.07,
            y=0.5,
            showarrow=False,
            text="y",
            textangle=-90,
            xref="paper",
            yref="paper"
        )
    ],
    autosize=True,
    margin=dict(
        b=100
    ),
    title_text="Ovisnost y o x")
fig.show()