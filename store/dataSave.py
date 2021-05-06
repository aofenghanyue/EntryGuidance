import pandas as pd


class DataSave:
    """
    数据存储类
    具体用法：
    新建对象db = DataSave()
    插入数据：db.update({...})
    """
    def __init__(self):
        self.data_frame = None

    def update(self, new_data):
        if self.data_frame is None:
            self.data_frame = pd.DataFrame([list(new_data.values())], columns=list(new_data.keys()))
        else:
            self.do_insert(new_data)

    def do_insert(self, data):
        exist_columns = self.data_frame.columns
        to_insert = {key: None for key in exist_columns}
        for key in data.keys():
            if key not in exist_columns:
                self.data_frame.insert(len(exist_columns), column=key, value=None)

        to_insert.update(data)
        self.data_frame.loc[len(self.data_frame.index)] = to_insert

    @property
    def data(self):
        return self.data_frame


if __name__ == '__main__':
    d = DataSave()
    d.update({'t': 2, 'k': 3})
    print(d.data_frame)
    d.update({'k': 3, 'j': 56})
    d.update({'k': 3, 'f': 56})
    print(d.data_frame.head(3))
