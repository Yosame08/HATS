import random
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd
import time

# 检查是否支持CUDA
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device: ", device)


# 加载数据
def load_data(file_path):
    data = pd.read_csv(file_path)
    X = data.drop('vel', axis=1).values
    y = data['vel'].values
    X = torch.tensor(X, dtype=torch.float)
    y = torch.tensor(y, dtype=torch.float)
    return X, y


print("Loading dataset...")
X_train, y_train = load_data('../data_vel_train.csv')
X_valid, y_valid = load_data('../data_vel_valid.csv')

print("Preparing dataset...")
# 创建数据加载器
batch_size = 4096  # 你可以根据需要调整这个值
train_data = TensorDataset(X_train, y_train)
train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True, num_workers=8)

input_size = X_train.shape[1]
output_size = 1

# 定义模型
model = nn.Sequential(
    #nn.Linear(input_size, 512), nn.ELU(),
    nn.Linear(input_size, 256), nn.ELU(),
    nn.Linear(256, 128), nn.ELU(),
    nn.Linear(128, 64), nn.ELU(),
    nn.Linear(64, 32), nn.ELU(),
    nn.Linear(32, 16), nn.ELU(),
    nn.Linear(16, 8), nn.ELU(),
    nn.Linear(8, output_size), nn.ELU(),
    # nn.Linear(4, output_size)
).to(device)

# 定义损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=2e-4, weight_decay=2e-5)
# scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=2, gamma=0.9)

print("Start training...")
valid_losses = []
# 训练模型
for epoch in range(300):  # 请根据你的需求调整迭代次数
    startTM = time.time()
    model.train()
    train_loss = 0
    for X_batch, y_batch in train_loader:
        X_batch = X_batch.to(device)
        y_batch = y_batch.to(device)
        optimizer.zero_grad()
        outputs = model(X_batch)
        loss_train = criterion(outputs.view(-1), y_batch)
        loss_train.backward()
        optimizer.step()
        train_loss += loss_train.item()
    train_loss /= len(train_loader)

    model.eval()
    with torch.no_grad():
        outputs_valid = model(X_valid.to(device))
        loss_valid = criterion(outputs_valid.view(-1), y_valid.to(device))
    # 随机选择一条验证数据
    idx = random.randint(0, len(X_valid) - 1)
    X_sample = X_valid[idx]
    y_sample = y_valid[idx]
    with torch.no_grad():
        output_sample = model(X_sample.unsqueeze(0).to(device))
    pred, act = output_sample.item(), y_sample.item()
    print(f'Random Sample: predict {pred:.2f}m/s, actual {act:.2f}m/s, delta = {pred-act:.2f}m/s')
    print(
        f'Epoch {epoch + 1}, Train Loss: {train_loss:.6f}, Valid Loss: {loss_valid.item():.6f}, Epoch Time: {time.time() - startTM:.2f}s')
    valid_losses.append(loss_valid.item())
    # 检查最近3个epoch的最低验证损失和最近4到6个epoch的最低验证损失
    if epoch >= 8:
        last_4_min_loss = min(valid_losses[-4:])
        last_5_to_8_min_loss = min(valid_losses[-8:-4])
        if last_4_min_loss >= 0.999 * last_5_to_8_min_loss:
            print("Training stopped.")
            break

model = model.to(torch.device("cpu"))  # 保存为CPU上的模型，HMM中使用CPU跑小数据
# 保存模型
torch.save(model.state_dict(), 'model_vel.pth')

# 保存模型
example_input = torch.rand(1, X_train.shape[1])  # 这是一个输入示例
traced_script_module = torch.jit.trace(model, example_input)

# 保存TorchScript模型
traced_script_module.save("model_vel.pt")
