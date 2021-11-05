# -*- coding: utf-8 -*-
# EditTime  : 2021-06-02 16:41
# Author    : Of yue
# File      : test_origin.py
# Intro     :

import originpro as op
import os

# Ensures that the Origin instance gets shut down properly.
import sys


def origin_shutdown_exception_hook(exctype, value, traceback):
    op.exit()
    sys.__excepthook__(exctype, value, traceback)


if op and op.oext:
    sys.excepthook = origin_shutdown_exception_hook

# Set Origin instance visibility.
if op.oext:
    op.set_show(True)

# YOUR CODE HERE
# op.set_show()
# x = [i for i in range(10)]
# y = [23, 45, 78, 133, 178, 199, 234, 278, 341, 400]
#
# work_sheet = op.new_sheet('w')
#
# work_sheet.from_list(0, x, '横坐标')
# work_sheet.from_list(1, y, '纵坐标')
#
# graph = op.new_graph('my_figure')
# sub_fig = graph[0]
# sub_fig.add_plot(work_sheet, 1, 0)
# sub_fig.rescale()
#
# store_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),'imgs/example.png')
# # store_path = '../imgs/example.png'  # op.path('e') + 'example.png'
# graph.save_fig(store_path)
# print(f'{sub_fig} is exported as {store_path}')
wks = op.new_sheet()
wks.from_file(os.path.join(op.path('e'), 'Samples', 'Graphing', 'Group.dat'))
graph = op.new_graph(template='scatter')
gl=graph[0]

# plot whole sheet as XY plot
plot = gl.add_plot(f'{wks.lt_range()}!(?,1:end)')
print(f'{wks.lt_range()}!(?,1:end)')

# group the plots and control plots setting in group
gl.group()
plot.colormap = 'Candy'
plot.shapelist = [3, 2, 1]
gl.rescale()


# Exit running instance of Origin.
if op.oext:
    input('按任意键结束')
    op.exit()