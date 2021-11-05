# -*- coding: utf-8 -*-
# EditTime  : 2021-06-02 15:52
# Author    : Of yue
# File      : origin.py
# Intro     :

import originpro as op
import os
import sys

import pandas
import pandas as pd


class OriginPlot:
    def __init__(self):
        def origin_shutdown_exception_hook(exctype, value, traceback):
            op.exit()
            sys.__excepthook__(exctype, value, traceback)

        if op and op.oext:
            sys.excepthook = origin_shutdown_exception_hook

        self.database = None
        self.database_df = None
        self.plot_series = []

        self.worksheet = None

    def set_database_df(self, df: pandas.DataFrame):
        self.database_df = df
        book_template = op.load_book(op.path('u') + r'Templates\trajectory.ogwu')
        self.worksheet = book_template[0]

    def add_plot_df(self, cols: list or str, row: str, template=None):
        def _add_plot(ax: op.graph.GLayer):
            pass

    def plot(self):
        if op.oext:
            op.set_show(True)
        self.worksheet.from_df(self.database_df)
        op.wait()
        op.wait('s', 0.2)

    def close(self):
        if op.oext:
            input('按任意键结束')
            op.exit()


def debug_origin(df: pandas.DataFrame):
    p = OriginPlot()
    p.set_database_df(df)
    p.plot()
    p.close()


if __name__ == '__main__':
    pass
