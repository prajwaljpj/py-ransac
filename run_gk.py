from pypcd import pypcd
import numpy as np
from gk_cluster import GK
import random

import plotly.graph_objs as go
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
import dash
import dash_html_components as html
import dash_core_components as dcc

# external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

# app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


n_clusters = 8
pc = pypcd.PointCloud.from_path('data/iisc/131.pcd')
xcoord = pc.pc_data['x']
ycoord = pc.pc_data['y']
zcoord = pc.pc_data['z']
data = np.stack((xcoord, ycoord, zcoord), axis=1) 
datan = data[~np.isnan(data).any(axis=1)]
g = GK(datan, n_clusters=n_clusters) 
fm, cf, ca = g.predict()
colors = []
for i in range(n_clusters):
    r = random.randint(0,255)
    g = random.randint(0,255)
    b = random.randint(0,255)
    colors.append((r,g,b))

color_code = ["rgb"+str(colors[a]) for a in ca]

trace1 = go.Scatter3d(
        x=xcoord,
        y=ycoord,
        z=zcoord,
        mode='markers',
        marker=dict(
            size=1,
            line=dict(
                color=color_code,
                width=0.5
                )
            )
        )

dat = [trace1]


plot(dat, filename="asdf.html")

