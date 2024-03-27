import numpy as np
import torch
import torch.nn as nn
import pandas as pd
from sklearn.preprocessing import MinMaxScaler

# 加载数据
data = pd.read_csv('your_file.csv')

# 数据预处理
scaler = MinMaxScaler()
data_scaled = data.groupby('id').transform(lambda x: scaler.fit_transform(x))

# 创建数据集
def create_dataset(data, look_back=1):
    dataX, dataY = [], []
    for i in range(len(data)-look_back-1):
        a = data[i:(i+look_back), :]
        dataX.append(a)
        dataY.append(data[i + look_back, -1])
    return np.array(dataX), np.array(dataY)

look_back = 3
trainX, trainY = create_dataset(data_scaled, look_back)

# 转换为张量
trainX = torch.from_numpy(trainX).float()
trainY = torch.from_numpy(trainY).float()

# 定义模型
class LSTM(nn.Module):
    def __init__(self, input_size=1, hidden_layer_size=100, output_size=1):
        super().__init__()
        self.hidden_layer_size = hidden_layer_size
        self.lstm = nn.LSTM(input_size, hidden_layer_size)
        self.linear = nn.Linear(hidden_layer_size, output_size)
        self.hidden_cell = (torch.zeros(1,1,self.hidden_layer_size),
                            torch.zeros(1,1,self.hidden_layer_size))

    def forward(self, input_seq):
        lstm_out, self.hidden_cell = self.lstm(input_seq.view(len(input_seq) ,1, -1), self.hidden_cell)
        predictions = self.linear(lstm_out.view(len(input_seq), -1))
        return predictions[-1]

# 训练模型
model = LSTM()
loss_function = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

epochs = 150

for i in range(epochs):
    model.train()
    optimizer.zero_grad()
    model.hidden_cell = (torch.zeros(1, 1, model.hidden_layer_size),
                         torch.zeros(1, 1, model.hidden_layer_size))

    y_pred = model(trainX)
    single_loss = loss_function(y_pred, trainY)
    single_loss.backward()
    optimizer.step()

    if i%25 == 1:
        print(f'epoch: {i:3} loss: {single_loss.item():10.8f}')

print(f'epoch: {i:3} loss: {single_loss.item():10.10f}')
