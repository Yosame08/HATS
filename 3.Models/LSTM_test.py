import torch
import torch.nn as nn
import seaborn as sns
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

os.environ["https_proxy"]="http://127.0.0.1:7890/"
os.environ["HTTPS_PROXY"]="http://127.0.0.1:7890/"
os.environ["HTTP_PROXY"]="http://127.0.0.1:7890/"
os.environ["http_proxy"]="http://127.0.0.1:7890/"

print("Import Finish")
print(sns.get_dataset_names())
flight_data = sns.load_dataset("flights")
print(flight_data.head())
print(flight_data.shape)
