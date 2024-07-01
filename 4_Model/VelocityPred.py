import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd
import time


def load_data(file_path):
    data = pd.read_csv(file_path)
    X = data.drop('vel', axis=1).values
    y = data['vel'].values
    X = torch.tensor(X, dtype=torch.float)
    y = torch.tensor(y, dtype=torch.float)
    return X, y


def one_epoch(device, model: nn.Sequential, optimizer, criterion, data_loader, is_train: bool):
    loss = 0
    if is_train:
        model.train()
    else:
        model.eval()
    for X_batch, y_batch in data_loader:
        X_batch = X_batch.to(device)
        y_batch = y_batch.to(device)
        optimizer.zero_grad()
        outputs = model(X_batch)
        loss_single = criterion(outputs.view(-1), y_batch)
        if is_train:
            loss_single.backward()
            optimizer.step()
        loss += loss_single.item()
    loss /= len(data_loader)
    return loss


def train(device, model: nn.Sequential, train_loader, valid_loader, best_loss, best_model):
    valid_losses = []
    criterion = nn.MSELoss()
    # jump out of local minimum
    optimizer = optim.Adam(model.parameters(), lr=1e-2, weight_decay=1e-6)
    train_loss = one_epoch(device, model, optimizer, criterion, train_loader, True)
    valid_loss = one_epoch(device, model, optimizer, criterion, valid_loader, False)
    print(f"Jump out of local minimum, valid loss = {valid_loss:.4f}")
    if valid_loss < best_loss:
        best_loss = valid_loss
        best_model = model.state_dict()
    optimizer = optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-6)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.75)
    for epoch in range(50):
        startTM = time.time()
        train_loss = one_epoch(device, model, optimizer, criterion, train_loader, True)

        model.eval()
        with torch.no_grad():
            valid_loss = one_epoch(device, model, optimizer, criterion, valid_loader, False)
        # randomly choose one
        # idx = random.randint(0, len(X_valid) - 1)
        # X_sample = X_valid[idx]
        # y_sample = y_valid[idx]
        # with torch.no_grad():
        #     output_sample = model(X_sample.unsqueeze(0).to(device))
        # pred, act = output_sample.item(), y_sample.item()
        # print(f'Random Sample: predict {pred:.2f}m/s, actual {act:.2f}m/s ({pred - act:.2f}m/s)')
        print(f'Epoch{epoch}({time.time() - startTM:.2f}s), Train Loss: {train_loss:.4f}, Valid Loss: {valid_loss:.4f}')
        valid_losses.append(valid_loss)
        # 如果当前验证损失比最小损失还小，更新最小损失并保存模型
        if valid_loss < best_loss:
            print(f"Best model updated: {best_loss:.4f} -> {valid_loss:.4f}")
            best_loss = valid_loss
            best_model = model.state_dict()  # 保存当前最佳模型
        # no loss decreasing in 4 continuous epochs: early stop
        if epoch >= 8:
            last_4_min_loss = min(valid_losses[-4:])
            last_5_to_8_min_loss = min(valid_losses[-8:-5])
            if last_4_min_loss >= last_5_to_8_min_loss:
                print(f"Early stopped at epoch {epoch} with loss {best_loss:.4f}")
                break
        scheduler.step()
    return best_loss, best_model


def main():
    best_loss = float('inf')
    best_model = None
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device: ", device)
    print("Loading dataset...")
    X_train, y_train = load_data('../Intermediate/data_vel_train.csv')
    X_valid, y_valid = load_data('../Intermediate/data_vel_valid.csv')

    print("Preparing dataset...")
    # 创建数据加载器
    batch_size = 8192  # 你可以根据需要调整这个值
    train_data = TensorDataset(X_train, y_train)
    valid_data = TensorDataset(X_valid, y_valid)
    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True, num_workers=14)
    valid_loader = DataLoader(valid_data, batch_size=batch_size, shuffle=False, num_workers=14)

    input_size = X_train.shape[1]
    output_size = 1

    # 定义模型
    model = nn.Sequential(
        nn.Linear(input_size, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
        nn.Linear(128, 64), nn.ELU(),
        nn.Linear(64, 32), nn.ELU(),
        nn.Linear(32, 16), nn.ELU(),
        nn.Linear(16, output_size),
    ).to(device)

    # 训练模型
    for times in range(3):
        print(f"Training #{times + 1}")
        best_loss, best_model = train(device, model, train_loader, valid_loader, best_loss, best_model)

    model = model.to(torch.device("cpu"))
    torch.save(best_model, '../Intermediate/model_vel.pth')
    # Save Model
    model.load_state_dict(best_model)
    example_input = torch.rand(1, X_train.shape[1])
    traced_script_module = torch.jit.trace(model, example_input)
    # Save TorchScript
    traced_script_module.save("../Intermediate/model_vel.pt")


if __name__ == '__main__':
    main()
